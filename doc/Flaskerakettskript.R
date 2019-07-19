library(readr)
library(haven)
library(readxl)
library(cli)
library(ggplot2)
library(reshape2)
library(dplyr)
library(tidyr)

Serial_A <- rename(serial_20190719_214517_A)
Serial_B <- rename(serial_20190719_214517_B)


Serial_A <- Serial_A[1:969,1:6]
View(Serial_A)   

Serial_B <- Serial_B[1:541,1:6]
View(Serial_B) 

hist(Serial_A$X5)
hist(Serial_B$X5)

plot(Serial_A$X2, Serial_A$X5,
     main = "Akselerasjon",
     ylab = "Akselerasjonsmagnitude", 
     xlab = "Tid i ms")

plot(Serial_B$X2, Serial_B$X5,
     main = "Trykk over tid",
     ylab = "Absolutt trykk i Pascal", 
     xlab = "Tid i ms")

Serial_alt <- serial_20190719_214517_tab
Serial_alt <- Serial_alt[5:1646,1:5]
View(Serial_alt)

Serial_wide <- spread(Serial_alt, X4, X5)
View(Serial_wide)

plot(Serial_B$X2, Serial_B$X5, type="b",
     main = "Trykk over tid",
     ylab = "Absolutt trykk i Pascal", 
     xlab = "Tid i ms")

plot(Serial_A$X2, Serial_A$X5,type="b",
     main = "Akselerasjon",
     ylab = "Akselerasjonsmagnitude", 
     xlab = "Tid i ms")

Akszoom<- Serial_A[378:678, 1:5]
View(Akszoom)

plot(Akszoom$X2, Akszoom$X5,type="b",
     main = "Akselerasjon",
     ylab = "Akselerasjonsmagnitude", 
     xlab = "Tid i ms")

Trykkzoom<- Serial_B[210:378, 1:5]
View(Trykkzoom)

plot(Trykkzoom$X2, Trykkzoom$X5,type="b",
     main = "Trykk over tid",
     ylab = "Absolutt trykk i Pascal", 
     xlab = "Tid i ms")