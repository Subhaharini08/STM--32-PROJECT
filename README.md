ML-Based Home Security and Safety System
Increasing concerns over home security due to rising burglary and unauthorized entry incidents. Conventional systems are often unable to detect specific audio cues indicating potential threats. Current solutions lack adaptive intelligence to detect unusual sounds like door breaking 
or dog barking.
PRINCIPLE:
Home Security and Safety:
Ensuring the safety of homes through advanced sound detection mechanisms.
Machine Learning in Security:
Enables real-time analysis, classification, and alerting based on audio signals.
Nano Edge AI:
An innovative AI technology optimized for edge devices. Provides adaptive learning and anomaly detection 
for audio signals without cloud dependency.
System Design Overview:

Sensors: Audio sensor (microphone), smoke sensor, gas sensor.
Microcontroller: STM32 (nucleoL432KC)

Algorithm Flow:

Step 1: Audio signal acquisition using the microphone.
Step 2: Preprocessing and feature extraction using NanoEdge AI.
Step 3: On-device training for anomaly detection (learning specific sounds).
Step 4: Classifying audio events (normal vs. suspicious).
Step 5: Triggering alerts (buzzer) if a threat is detected.

