class Configuration:
	name = ""
	hand_size = -1
	basic_mode = ""

	move = ""
	click = ""
	grab = ""
	lclick = ""
	scroll = ""
	zoom = ""
	
	close_w = ""
	minimize_w = ""
	maximize_w = ""

	# etc
	
def read_file(file):
	for c, line in enumerate(file):
		line = line.rstrip()
		if "name" in line:
			print(line)

file = open("conf_file.txt", "r")
read_file(file)
file.close()
