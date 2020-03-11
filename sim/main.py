'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Mar 08 2020
                                      SARC

  File Name  : main.py
  Description: Entry point for sim
---*-----------------------------------------------------------------------*'''

import env
import poi
import bs
import drone

e = env.env()
e.setup()

for i in range(100):
  e.tick()