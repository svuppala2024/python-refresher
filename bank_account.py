class BankAccount:
    def __init__(self, name, account_number, balance=0):
        self.balance = balance
        self.name = name
        self.account_number = account_number

    def withdraw(self, amount):
        if amount < 0:
            print(f"{amount} is not a valid amount to withdraw.")
        if amount > self.balance:
            print(
                "You don't have enough balance to withdraw " + str(amount) + " dollars."
            )
            pass
        self.balance -= amount

    def deposit(self, amount):
        if amount < 0:
            print(f"{amount} is not a valid amount to deposit.")
        self.balance += amount

    def current_balance(self):
        print(f"Your current balance is {self.balance}.")
