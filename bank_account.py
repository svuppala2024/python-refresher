class bank_account:
    def __init__(self, balance, name, account_number):
        self.balance = balance
        self.name = name
        self.account_number = account_number

    def withdraw(self, amount):
        if amount > self.balance:
            print(
                "You don't have enough balance to withdraw " + str(amount) + " dollars."
            )
            pass
        self.balance -= amount

    def deposit(self, amount):
        self.balance += amount

    def current_balance(self):
        print(str(self.balance))
