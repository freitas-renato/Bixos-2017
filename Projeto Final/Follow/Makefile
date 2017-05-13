#Processo Seletivo Thunderatz 2017
#Makefile AVR

#Segui os exemplos das aulinhas e o makefile real da equipe
#Pode ser que esteja faltando alguma coisa e tenha coisas erradas
####COMENTEM!####

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))

.PHONY: all clean flash

#Deixei os comandos aparecendo, colocar @ para "tirar"

all: main.hex

%.o: %.c
	@echo "Compilando..."
	avr-gcc -c $< -o $@ -mmcu=m8 -std=c99 -Wall -Wextra -Os

clean:
	@echo "Removendo arquivos..."
	rm -f $(OBJECTS) *.hex *.elf

flash:	
	@echo "Gravando na placa..."
	avrdude -c usbasp -p m168 -U flash:w:main.hex:i

help:
	@echo "Makefile para AVR - Processo Seletivo ThundeRatz"
	@echo "Comandos: "
	@echo "all: 	compila todos os arquivos .c da pasta"
	@echo "clean: 	remove os arquivos .o .hex e .elf"
	@echo "flash: 	grava os arquivos no microcontrolador"

main.elf: $(OBJECTS)
	@echo "Compilando main.elf..."
	avr-gcc -o main.elf $(OBJECTS) -mmcu=m8 -std=c99 -Wall -Wextra -Os

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
