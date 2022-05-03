Run as follows

python spanner.py InputFileName PositionFileName initialKValue iterations mode

where inputfilename is a txt file like 12node_edges.txt
positionfilename is a txt file like 12node_pos.txt
initialKvalue is a float value probably between 1 and 2
iterations is the number of times to increment from the intial k value by intervals of 0.1
mode is either 'd' or 'b' for dijkstras or bfs

example
python spanner.py 12node_edges.txt 12node_pos.txt 1.5 5 b