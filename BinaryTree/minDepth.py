class Node:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None

def minDepth(root):
    '''Takes the given argument as root and finds the shortest height from a leaf node.
        This is an example of Level Order Traversal.'''
    if root.value == None:
        return 0
    q = []
    q.append({'node': root, 'depth': 1})
    steps = 0
    while(len(q)>0):
        steps += 1
        temp = q.pop(0)
        node = temp['node']
        depth = temp['depth']
        if node.left is None and node.right is None:
            print("Steps: {}".format(steps))
            return depth
        if node.left is not None:
            q.append({'node': node.left, 'depth': depth + 1})
        if node.right is not None:
            q.append({'node': node.right, 'depth': depth + 1})
    

# Driver program to test above function 
# Lets construct a binary tree shown in above diagram 
root = Node(1) 
root.left = Node(2) 
root.right = Node(3) 
root.left.left = Node(4) 
root.left.right = Node(5) 
root.left.left.left = Node(4) 
root.left.left.right = Node(4) 
root.left.right.left = Node(5)
root.left.right.right = Node(5)
root.left.left.left.right = Node(4) 
root.left.right.right.left = Node(5)
print("Minimum depth to a leaf node: {}".format(minDepth(root)))


