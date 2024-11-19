import random

game_table = [
    [" ", " ", " "],
    [" ", " ", " "],
    [" ", " ", " "],
]

def showTable():
    for row in game_table:
        print(row)

def addItem(character, game_table, row, column):
    if game_table[row][column] not in ["X", "x", "O", "o"]:
        game_table[row][column] = character
    else:
        print("The square is full. Choose another row and column.")
        while True:
            try:
                row = int(input("Enter row (0, 1, or 2): "))
                column = int(input("Enter column (0, 1, or 2): "))
                if 0 <= row <= 2 and 0 <= column <= 2:
                    if game_table[row][column] not in ['X', 'x', 'O', 'o']:
                        game_table[row][column] = character
                        break
                    else:
                        print("The square is full. Choose another one.")
                else:
                    print("Invalid input. Row and column must be between 0 and 2.")
            except ValueError:
                print("Invalid input. Please enter valid integers for row and column.")

def computer_turn(character_of_player, game_table):
    computer_character = "O" if character_of_player.upper() == "X" else "X"
    row_to_add, column_to_add = random.randint(0, 2), random.randint(0, 2)
    while game_table[row_to_add][column_to_add] in ["X", "O"]:
        row_to_add, column_to_add = random.randint(0, 2), random.randint(0, 2)
    game_table[row_to_add][column_to_add] = computer_character

def whoIsWinner(game_table, computer_value, player_value):
    for i in range(3):
        if game_table[i][0] == game_table[i][1] == game_table[i][2] != " ":
            value = game_table[i][0]
            print(f"Game is over. {'Computer' if value == computer_value else 'You'} WIN.\n")
            return True
    for j in range(3):
        if game_table[0][j] == game_table[1][j] == game_table[2][j] != " ":
            value = game_table[0][j]
            print(f"Game is over. {'Computer' if value == computer_value else 'You'} WIN.\n")
            return True
    if game_table[0][0] == game_table[1][1] == game_table[2][2] != " ":
        value = game_table[0][0]
        print(f"Game is over. {'Computer' if value == computer_value else 'You'} WIN.\n")
        return True
    if game_table[0][2] == game_table[1][1] == game_table[2][0] != " ":
        value = game_table[0][2]
        print(f"Game is over. {'Computer' if value == computer_value else 'You'} WIN.\n")
        return True
    return False

print("Hello :)")
value = input("Which character would you like to play X or O: ")
while value not in ['X', 'O', 'x', 'o']:
    value = input("Invalid choice! Please choose 'X' or 'O': ")

comp_value = "O" if value.upper() == "X" else "X"

while True:
    print("\nYour turn!")
    row = int(input("Enter row (0, 1, or 2): "))
    column = int(input("Enter column (0, 1, or 2): "))
    addItem(value.upper(), game_table, row, column)
    showTable()

    if whoIsWinner(game_table, comp_value, value.upper()):
        break
    if all(cell != " " for row in game_table for cell in row):
        print("It's a tie!")
        break

    computer_turn(value.upper(), game_table)
    print("\nComputer's turn:")
    showTable()

    if whoIsWinner(game_table, comp_value, value.upper()):
        break
    if all(cell != " " for row in game_table for cell in row):
        print("It's a tie!")
        break
