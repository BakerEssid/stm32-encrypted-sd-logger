# 🛡️ STM32 Encrypted SD Logger

Ce projet consiste à développer un système embarqué de stockage sécurisé de données sur carte microSD,
basé sur un microcontrôleur STM32F407VGTx. Les données issues de capteurs (ADC, RTC) sont chiffrées avec
AES en mode CBC avant d’être enregistrées sur la carte SD. Le projet utilise FreeRTOS pour gérer les tâches
concurrentes et le système de files de messages pour la communication inter-tâches.

# 📌 Objectifs du projet

**.** Collecter des données en temps réel depuis des capteurs ADC et RTC.

**.** Chiffrer les données avec AES-128/192 en mode CBC pour assurer la confidentialité.

**.** Stocker les données chiffrées sur microSD avec le système de fichiers FATFS.

**.** Gérer les tâches en parallèle avec FreeRTOS pour :

    - Acquisition ADC

    - Lecture RTC

    - Chiffrement des données

    - Écriture sur la carte SD

**.** Fournir une structure modulable pour l’ajout futur de nouveaux capteurs ou fonctionnalités.
