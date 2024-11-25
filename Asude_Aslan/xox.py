def show_game_board(game_board):
    for row in game_board:
        print(" | ".join(row))
        print("-" * 9)

def create_game_board():
    return [[" " for _ in range(3)] for _ in range(3)]

def make_move(game_board, player, row, column):
    if game_board[row][column] == " ":
        game_board[row][column] = player
        return True
    else:
        print("This area is already full.Please choose another area!")
        return False

def is_the_winner(game_board, player):
    for i in range(3):
        if all(game_board[i][j] == player for j in range(3)):
            return True

    for j in range(3):
        if all(game_board[i][j] == player for i in range(3)):
            return True

    if all(game_board[i][i] == player for i in range(3)) or all(game_board[i][2 - i] == player for i in range(3)):
        return True

    return False

def game():
    game_board = create_game_board()
    turn = "X"
    winner = None

    while True:
        show_game_board(game_board)
        print(f"Turn: {turn}")
        row = int(input("Choose row (0, 1, 2): "))
        column = int(input("Choose column (0, 1, 2): "))

        if row < 0 or row > 2 or column < 0 or column > 2:
            print("Invalid move! The row and column must be 0 , 1 or 2.")
            continue

        if make_move(game_board, turn, row, column):
            if is_the_winner(game_board, turn):
                winner = turn
                break
            elif " " not in [row for column in game_board for row in column]:
                break
            turn = "O" if turn == "X" else "X"

    show_game_board(game_board)
    if winner:
        print(f"{winner} won!")
    else:
        print("The game is tied!")

if __name__ == "__main__":
    game()