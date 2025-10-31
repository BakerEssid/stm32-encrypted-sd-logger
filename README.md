# 💾 STM32 Encrypted SD Logger

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

# 🔒 Exemple de données chiffrées


# 🤝 Contribution
Les contributions sont les bienvenues !
N’hésitez pas à ouvrir une issue ou à proposer une pull request pour améliorer le projet.

# 📧 Contact
Pour toute question, suggestion ou collaboration, vous pouvez me contacter via :

Email : baker.essid98@gmail.com

LinkedIn : [Baker Essid](https://www.linkedin.com/in/baker-essid-b27b311b9/overlay/about-this-profile/?lipi=urn%3Ali%3Apage%3Ad_flagship3_profile_view_base%3Bgh8EYV5MTL%2BDU11rWtcMPA%3D%3D)

Je répondrai avec plaisir à vos messages et discussions autour du projet.
