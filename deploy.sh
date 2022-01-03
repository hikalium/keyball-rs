#!/bin/bash -xe
ELF_PATH="$1"
if [ -z "${SSH_OPENOCD}" ]
then
	elf2uf2-rs -d "${ELF_PATH}"
else
	# `SSH_OPENOCD=pi4 cargo run --release` to deploy the binary via openocd on a host ${SSH_OPENOCD}
	scp ${ELF_PATH} ${SSH_OPENOCD}:~/target.elf && \
		ssh ${SSH_OPENOCD} -- openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c '"targets rp2040.core0; program target.elf verify reset exit"'
fi
