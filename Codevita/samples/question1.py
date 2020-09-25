'''
Some of the keys of Ajith's Laptop's Keyboard are damaged and he is not able to type those keys. He has to complete his assignment and submit it the next day and since it is midnight he will not be able to give his laptop for repair. So he decides to make a character sequence of all the damaged keys in a sequence that he can copy and paste and make a word out of them.

Ajith needs to type a paragraph with all the characters in lower case. Help Ajith to find out the best permutation of the sequence of the characters (corresponding to the damaged keys) per word, that can be used while typing the paragraph, i.e. the sequence that will require least insertion and deletion while typing a word. Consider paste operation to be of one keystroke. Ignore the copy operation.

Recursively apply the same procedure for all the words in the paragraph. This way you will get the best combination that should be selected for that word. Finally, how many different words exist per character sequence combination. The combination that is the best for maximum words should be printed as output. If there are more than one candidates for best character sequence print the lexicographically smallest character sequence.

Refer the example section for better understanding.

Constraints
0 < Number of words in paragraph < 50

0 < Number of damaged keys <= 6

Input
First line contains the paragraph P that is to be written Second line contains the characters that represent the damaged keys

Output
One Line containing the best string which can be used to copy paste for the words.

Time Limit
1
'''