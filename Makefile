all:	
	@make -C qualisys/common
	@make -C qualisys

clean:
	@make -C qualisys/common -s clean
	@make -C qualisys -s clean
