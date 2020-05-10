#!/usr/bin/expect -f

set prompt "#"
set address "A4:53:85:4C:0F:32"

spawn bluetoothctl
expect -re $prompt
send "power on\r"
expect -re $prompt
send "agent on\r"
expect -re $prompt
send "default-agent\r"
expect -re $prompt
send "scan on\r"
expect -re $prompt
send "remove $address\r"
sleep 1
expect -re $prompt
send "trust $address\r"
send_user "\nSleeping\r"
sleep 5
send_user "\nDone sleeping\r"
expect -re $prompt
send "pair $address\r"
expect -re $prompt
send "quit\r"
expect eof
