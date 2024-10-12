#!/usr/bin/env python3



class FixedList:
    def __init__(self, size):
        self.size = size
        self.data = []

    def is_full(self):
        return len(self.data) == self.size

    def append(self, item):
        if self.is_full():
            self.data.pop(0)  # Remove the oldest element
        self.data.append(item)

    def __getitem__(self, index):
        return self.data[index]

    def __len__(self):
        return len(self.data)

    def __str__(self):
        return str(self.data)
    

