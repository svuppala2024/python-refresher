import unittest
import bank_account


class TestBankAccount(unittest.TestCase):
    def test_init(self):
        b1 = bank_account.BankAccount("Suhruth", 12039, 0)

        self.assertEqual(b1.name, "Suhruth")
        self.assertEqual(b1.account_number, 12039)
        self.assertEqual(b1.balance, 0)

        self.assertNotEqual(b1.name, "Bob")
        self.assertNotEqual(b1.account_number, 0)
        self.assertNotEqual(b1.balance, 1)

    def test_deposit(self):
        b1 = bank_account.BankAccount("Suhruth", 12039, 0)
        b1.deposit(500)
        self.assertEqual(b1.balance, 500)
        self.assertNotEqual(b1.balance, 0)

    def test_withdraw(self):
        b1 = bank_account.BankAccount("Suhruth", 12039, 500)
        b1.withdraw(500)
        self.assertEqual(b1.balance, 0)
        self.assertNotEqual(b1.balance, 1000)


if __name__ == "__main__":
    unittest.main()
