import random
board=["-","-","-",
       "-","-","-",
       "-","-","-"]
currentPlayer= "X"
winner= None
game_running= True
def printBoard(board):
    print(board[0]+"|"+board[1]+"|"+board[2]+"|")
    print("-"*6)
    print(board[3]+"|"+board[4]+"|"+board[5]+"|")
    print("-" * 6)
    print(board[6] + "|" + board[7] + "|" + board[8] + "|")

#take input from player
def playerInput(board):
    inp= int(input("Enter a number between 1-9: "))
    if inp >= 1 and inp <= 9 and board[inp-1] == "-":
        board[inp-1]= currentPlayer
    else:
        print("Please enter an available number")
        printBoard(board)
        playerInput(board)

#check for win or tie
def checkHorizantle(board):
    global winner
    if board[0]==board[1]==board[2] and board[0] != "-":
        winner = board[0]
        return True
    elif board[3]==board[4]==board[5] and board[3] != "-":
        winner = board[3]
        return True
    elif board[6]==board[7]==board[8] and board[6] != "-":
        winner = board[6]
        return True
    return False
def checkVertical(board):
    global winner
    if board[0] == board[3] == board[6] and board[3] != "-":
        winner = board[0]
        return True
    elif board[1] == board[4] == board[7] and board[1] != "-":
        winner = board[1]
        return True
    elif board[2] == board[5] == board[8] and board[2] != "-":
        winner = board[2]
        return True
    return False
def checkCross(board):
    global winner
    if board[0]==board[4]==board[8] and board[0] != "-":
        winner= board[0]
        return True
    elif board[2]==board[4]==board[6] and board[2] != "-":
        winner= board[2]
        return True
    return False

def checkwin():
    global game_running
    if game_running:

        if checkCross(board) or checkHorizantle(board) or checkVertical(board):
            printBoard(board)
            print(f"the winner is {winner}")
            game_running = False
            return True
    return False

def checkTie(board):
    global game_running
    if  winner is None and "-" not in board:
        printBoard(board)
        print("it is a tie")
        game_running= False
        return True
    return False


#changing current player
def changePlayer():
    global currentPlayer
    if currentPlayer== "X":
        currentPlayer = "O"
    else:
        currentPlayer="X"
# computer plays
def compGame(board):
    while currentPlayer == "O":
        position = random.randint(0,8)
        if board[position] == "-" :
            board[position] = "O"
            changePlayer()

while game_running:
    printBoard(board)
    if currentPlayer== "X":
        playerInput(board)
        changePlayer()
    else:
        compGame(board)
    if checkwin() or checkTie(board):
        break