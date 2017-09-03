package main

import (
	"fmt"
	"log"
	"io"
//    "encoding/hex"
	"github.com/tarm/serial"
	"bytes"
	"time"
)

func main() {
	c := &serial.Config{Name: "/dev/ttyUSB0", Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}
	defer s.Close()

	_, err = s.Write([]byte("T"))
	if err != nil {
		log.Fatal(err)
	}

	buf := make([]byte, 0, 256)
	temp_buf := make([]byte, 256)
	for {
		time.Sleep(10 * time.Millisecond)
		n, err := s.Read(temp_buf)
// fmt.Println("Read: ", n, err)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Error reading from serial port: ", err)
			}
		} else {
			// log.Fatal(err)
			// temp_buf = temp_buf[:n]
			fmt.Println("Rx: ", string(temp_buf[:n]))
			// fmt.Println("Rx: ", hex.EncodeToString(temp_buf[:n]))
			buf = append(buf, temp_buf[:n]...);
			fmt.Println("Buf: ", string(buf))
			newline := false
			// p := bytes.Index(buf, []byte("\r\n"))
			// p := bytes.Index(buf, []byte("\n"))
			p := bytes.Index(buf, []byte("\x03"))
			for p != -1 {
fmt.Println(len(buf), cap(buf), p)
				newline = true
				fmt.Println(">>> ", string(buf[:p]))
				buf = buf[p+1:]
				p = bytes.Index(buf, []byte("\n"))
			}
			if newline {
				time.Sleep(5 * time.Second)
				_, err := s.Write([]byte("T"))
				if err != nil {
					log.Fatal(err)
				}
			}
		}
	}

//        log.Printf("%q", buf[:n])
}