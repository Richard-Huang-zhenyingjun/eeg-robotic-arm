from mock_serial import MockSerial

ser = MockSerial()
ser.write(b"L\n")
print(ser.readline())
