#Processo Seletivo Thunderatz 2017
#Makefile AVR

#Segui os exemplos das aulinhas e o makefile real da equipe
#Pode ser que esteja faltando alguma coisa e tenha coisas erradas
####COMENTEM!####

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))

.PHONY: all clean flash main.elf main.hex #nao sei se posso/preciso colocar os dois "main." aqui


#Deixei os comandos aparecendo, colocar @ para "tirar"
#Adicionar um 'help' (?)

all: main.hex

clean:
	@echo "Removendo arquivos..."
	rm -f $(OBJECTS) *.hex *.elf

%.o: %$(SOURCES)
	@echo "Compilando..."
	avr-gcc -c $< -o $@ -mmcu=m8 -std=c99 -Wall -Wextra -Os

flash:	
	@echo "Gravando na placa..."
	avrdude -c usbasp -p m168 -U flash:w:main.hex:i

main.elf: $(OBJECTS)
	@echo "Compilando main.elf..."
	avr-gcc -o main.elf $(OBJECTS) -mmcu=m8 -std=c99 -Wall -Wextra -Os

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex