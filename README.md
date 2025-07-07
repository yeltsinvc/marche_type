# Marche Type Example

Este repositorio contiene un ejemplo simple para simular la marcha tipo de un BRT basándose en un modelo de seguimiento de vehículos (car-following). Utiliza el Modelo de Conductor Inteligente (IDM) para calcular la aceleración del vehículo seguidor.

El script `marche_type.py` genera una lista de posiciones y velocidades para el vehículo a intervalos de medio segundo. Se puede ejecutar directamente con Python:

```bash
python3 marche_type.py
```

El resultado es una serie de pares `distancia, velocidad` que representan la trayectoria simulada.
