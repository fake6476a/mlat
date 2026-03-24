package main

import (
	"context"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	sdk "github.com/NeuronInnovations/neuron-go-hedera-sdk"
	commonlib "github.com/NeuronInnovations/neuron-go-hedera-sdk/common-lib"
	"github.com/hashgraph/hedera-sdk-go/v2"
	"github.com/libp2p/go-libp2p/core/host"
	"github.com/libp2p/go-libp2p/core/network"
	"github.com/libp2p/go-libp2p/core/protocol"
)

var Version = "0.1.0"

const ADSBProtocol = protocol.ID("neuron/ADSB/0.0.2")

// ModeSPacket represents a single parsed Mode-S reception from a sensor
type ModeSPacket struct {
	SensorID    int64   `json:"sensor_id"`
	Latitude    float64 `json:"lat"`
	Longitude   float64 `json:"lon"`
	Altitude    float64 `json:"alt"`
	TimestampS  uint64  `json:"timestamp_s"`
	TimestampNS uint64  `json:"timestamp_ns"`
	RawMessage  string  `json:"raw_msg"`
}

var (
	packetCount   atomic.Int64
	activeSensors sync.Map
)

func main() {
	log.SetOutput(os.Stderr)
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	log.Println("=== MLAT Data Pipe v" + Version + " ===")
	log.Println("Starting Neuron SDK buyer node...")
	log.Println("Mode-S packets will be emitted as JSON on stdout")

	// Inject only the flags that LaunchSDK() still reads after init; early SDK flags must stay on the CLI.
	injectCLIFlags()

	go logStats()

	sdk.LaunchSDK(
		Version,
		ADSBProtocol,
		nil,
		buyerCase,
		buyerTopicListener,
		nil,
		nil,
	)
}

// buyerCase sets a stream handler so that when sellers open streams to us, we read Mode-S data.
func buyerCase(ctx context.Context, p2pHost host.Host, buffers *commonlib.NodeBuffers) {
	log.Println("Buyer case started - setting up stream handler for /adsb/v1")

	p2pHost.SetStreamHandler(ADSBProtocol, func(stream network.Stream) {
		remotePeer := stream.Conn().RemotePeer()
		log.Printf("NEW STREAM from seller: %s", remotePeer.String())
		activeSensors.Store(remotePeer.String(), time.Now())
		go readSellerStream(ctx, stream)
	})

	log.Println("Stream handler registered. Waiting for seller connections...")

	go func() {
		ticker := time.NewTicker(60 * time.Second)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				count := 0
				activeSensors.Range(func(key, value interface{}) bool {
					count++
					return true
				})
				log.Printf("Active sensor connections: %d", count)
			}
		}
	}()

	<-ctx.Done()
	log.Println("Buyer case context cancelled, shutting down")
}

// readSellerStream reads length-prefixed seller packets carrying sensor metadata and raw Mode-S payload bytes.
func readSellerStream(ctx context.Context, stream network.Stream) {
	defer stream.Close()
	remotePeer := stream.Conn().RemotePeer()
	peerStr := remotePeer.String()

	log.Printf("Reading data from seller: %s", peerStr)

	for {
		select {
		case <-ctx.Done():
			return
		default:
		}

		lengthBuf := make([]byte, 1)
		_, err := io.ReadFull(stream, lengthBuf)
		if err != nil {
			if err == io.EOF {
				log.Printf("Stream from %s closed (EOF)", peerStr)
			} else {
				log.Printf("Error reading length from %s: %v", peerStr, err)
			}
			activeSensors.Delete(peerStr)
			return
		}

		packetLen := int(lengthBuf[0])
		if packetLen == 0 {
			continue
		}

		packetBuf := make([]byte, packetLen)
		_, err = io.ReadFull(stream, packetBuf)
		if err != nil {
			log.Printf("Error reading packet from %s: %v", peerStr, err)
			activeSensors.Delete(peerStr)
			return
		}

		packet, err := parsePacket(packetBuf)
		if err != nil {
			count := packetCount.Load()
			if count%100 == 0 {
				log.Printf("Parse error (every 100th): %v (len=%d)", err, packetLen)
			}
			continue
		}

		jsonBytes, err := json.Marshal(packet)
		if err != nil {
			log.Printf("Error marshaling JSON: %v", err)
			continue
		}

		fmt.Println(string(jsonBytes))

		count := packetCount.Add(1)
		if count%1000 == 0 {
			log.Printf("Total packets received: %d", count)
		}

		activeSensors.Store(peerStr, time.Now())
	}
}

// parsePacket decodes the binary packet layout into a ModeSPacket.
func parsePacket(data []byte) (*ModeSPacket, error) {
	if len(data) < 55 {
		return nil, fmt.Errorf("packet too short: %d bytes (need at least 55)", len(data))
	}

	packet := &ModeSPacket{
		SensorID:    int64(binary.BigEndian.Uint64(data[0:8])),
		Latitude:    math.Float64frombits(binary.BigEndian.Uint64(data[8:16])),
		Longitude:   math.Float64frombits(binary.BigEndian.Uint64(data[16:24])),
		Altitude:    math.Float64frombits(binary.BigEndian.Uint64(data[24:32])),
		TimestampS:  binary.BigEndian.Uint64(data[32:40]),
		TimestampNS: binary.BigEndian.Uint64(data[40:48]),
	}

	if len(data) > 48 {
		packet.RawMessage = hex.EncodeToString(data[48:])
	}

	if packet.Latitude < -90 || packet.Latitude > 90 {
		return nil, fmt.Errorf("invalid latitude: %f", packet.Latitude)
	}
	if packet.Longitude < -180 || packet.Longitude > 180 {
		return nil, fmt.Errorf("invalid longitude: %f", packet.Longitude)
	}

	return packet, nil
}

func buyerTopicListener(topicMessage hedera.TopicMessage) {
	log.Printf("Hedera topic message: %s", string(topicMessage.Contents))
}

func logStats() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()
	for range ticker.C {
		count := packetCount.Load()
		sensorCount := 0
		activeSensors.Range(func(key, value interface{}) bool {
			sensorCount++
			return true
		})
		log.Printf("Stats: packets=%d, active_sensors=%d", count, sensorCount)
	}
}

// injectCLIFlags only adds fallback flags that LaunchSDK() reads after SDK init, because early SDK flags must be passed on the command line.
func injectCLIFlags() {
	hasFlag := func(name string) bool {
		for _, arg := range os.Args[1:] {
			if strings.Contains(arg, name) {
				return true
			}
		}
		return false
	}

	// Inject `--list-of-sellers-source` here because LaunchSDK() still reads it after init.
	if !hasFlag("list-of-sellers-source") {
		os.Args = append(os.Args, "--list-of-sellers-source", "env")
	}
}

// readEnvFile parses a simple KEY=VALUE env file (no quotes handling needed).
func readEnvFile(path string) map[string]string {
	result := make(map[string]string)
	data, err := os.ReadFile(path)
	if err != nil {
		return result
	}
	for _, line := range strings.Split(string(data), "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		parts := strings.SplitN(line, "=", 2)
		if len(parts) == 2 {
			result[strings.TrimSpace(parts[0])] = strings.TrimSpace(parts[1])
		}
	}
	return result
}
