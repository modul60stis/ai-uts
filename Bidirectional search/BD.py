# Python3 program for Bidirectional BFS 
# Search to check jalur between two vertices

# Pendefinisian Class dari node
# untuk ditambahkan pada graph
class AdjacentNode:
	
	def __init__(self, vertex):
		
		self.vertex = vertex
		self.next = None

# Implementasi BidirectionalSearch
class BidirectionalSearch:
	
	def __init__(self, jumlahNode):
		
		# Simpan jumlah node
		# siapkan list untuk menyimpan node
		self.jumlahNode = jumlahNode
		self.graph = [None] * self.jumlahNode
		
		# Inisialisasi queue untuk forward search 
		# dan backward search
		self.src_queue = list()
		self.dest_queue = list()
		
		# Inisialisasi node yang telah dikunjungi dari forward dan 
		# backward search as False
		self.src_visited = [False] * self.jumlahNode
		self.dest_visited = [False] * self.jumlahNode
		
		# Inisialisasi parent node dari source 
		# dan destination
		self.src_parent = [None] * self.jumlahNode
		self.dest_parent = [None] * self.jumlahNode
		
	# Fungsi untuk menambahkan undirected edge/garis
	def tambah_garis(self, src, dest): 
		
		# Menambahkan edge pada graf
		
		# Tambah source ke destination
		node = AdjacentNode(dest) 
		node.next = self.graph[src] 
		self.graph[src] = node 

		# Karena graph bersifat undirected, tambah
		# destination ke source
		node = AdjacentNode(src)
		node.next = self.graph[dest]
		self.graph[dest] = node
		
	# Fungsi untuk Breadth First Search 
	def bfs(self, direction = 'forward'):
		
		if direction == 'forward':
			
			# BFS secara forward
			current = self.src_queue.pop(0)
			connected_node = self.graph[current]
			
			while connected_node:
				vertex = connected_node.vertex
				
				if not self.src_visited[vertex]:
					self.src_queue.append(vertex)
					self.src_visited[vertex] = True
					self.src_parent[vertex] = current
					
				connected_node = connected_node.next
		else:
			
			# BFS secara backward
			current = self.dest_queue.pop(0)
			connected_node = self.graph[current]
			
			while connected_node:
				vertex = connected_node.vertex
				
				if not self.dest_visited[vertex]:
					self.dest_queue.append(vertex)
					self.dest_visited[vertex] = True
					self.dest_parent[vertex] = current
					
				connected_node = connected_node.next
				
	# Cek apakah ada vertex yang berpotongan dari
	# backward search dan forward search 
	def is_intersecting(self):
		
		# Mengembalikan pada node mana perpotongannya
		# jika tidak ada maka kembalikan 1
		for i in range(self.jumlahNode):
			if (self.src_visited[i] and
				self.dest_visited[i]):
				return i
				
		return -1

	# Print jalur dari sumber ke target
	def print_jalur(self, intersecting_node, 
				src, dest):
						
		# Print jalur akhir dari 
		# source ke destination
		jalur = list()
		jalur.append(intersecting_node)
		i = intersecting_node
		
		while i != src:
			jalur.append(self.src_parent[i])
			i = self.src_parent[i]
			
		jalur = jalur[::-1]
		i = intersecting_node
		
		while i != dest:
			jalur.append(self.dest_parent[i])
			i = self.dest_parent[i]
			
		print("***** Jalur *****")
		jalur = list(map(str, jalur))
		
		print(' '.join(jalur))
	
	# Fungsi untuk bidirectional search 
	def bidirectional_search(self, src, dest):
		
		# Tambah source ke queue dan tandai 
		# visited sebagai True dan tambahkan parent 
		# sebagai -1
		self.src_queue.append(src)
		self.src_visited[src] = True
		self.src_parent[src] = -1
		
		# Tambah destination ke queue dan tandai 
		# visited sebagai True dan tambahkan parent
		# sebagai -1
		self.dest_queue.append(dest)
		self.dest_visited[dest] = True
		self.dest_parent[dest] = -1

		while self.src_queue and self.dest_queue:
			
			# BFS dari arah forward direction
			self.bfs(direction = 'forward')
			
			# BFS dari arah berlawanan
			self.bfs(direction = 'backward')
			
			# Cek node yang berpotongan
			intersecting_node = self.is_intersecting()
			
			# Jika node berpotonagan ada
			# maka jalur dari source ke destination ada
			if intersecting_node != -1:
				print(f"jalur ada antara {src} dan {dest}")
				print(f"Berpotongan pada : {intersecting_node}")
				self.print_jalur(intersecting_node, 
								src, dest)
				exit(0)
		return -1

# Driver code
if __name__ == "__main__":
	
	# Benyak node di dalam graph
	n = 18
	
	# Start Node
	src = 3
	
	# End Node
	dest = 15
	
	# Buat Graph
	graph = BidirectionalSearch(n)
	graph.tambah_garis(0, 2)
	graph.tambah_garis(1, 2)
	graph.tambah_garis(2, 3)
	graph.tambah_garis(3, 4)
	graph.tambah_garis(3, 5)
	graph.tambah_garis(5, 6)
	graph.tambah_garis(5, 7)
	graph.tambah_garis(5, 8)
	graph.tambah_garis(8, 9)
	graph.tambah_garis(8, 10)
	graph.tambah_garis(10, 11)
	graph.tambah_garis(11, 12)
	graph.tambah_garis(11, 13)
	graph.tambah_garis(10, 14)
	graph.tambah_garis(14, 15)
	graph.tambah_garis(15, 16)
	graph.tambah_garis(14, 17)


	
	out = graph.bidirectional_search(src, dest)
	
	if out == -1:
		print(f"Tidak ada jalur penghubung antara node ke-{src} and ke-{dest}")


