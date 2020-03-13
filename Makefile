all:
	cc main.c -o astar

run: all
	./astar

clean:
	rm -f ./astar
