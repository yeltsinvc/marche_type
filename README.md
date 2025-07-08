# Marche Type Example

Este repositorio contiene un ejemplo simple para simular la marcha tipo de un BRT basándose en un modelo de seguimiento de vehículos (car-following). Utiliza el Modelo de Conductor Inteligente (IDM) para calcular la aceleración del vehículo seguidor.

El código se organiza en varios módulos y puede ejecutarse con `main.py`.
Para obtener la trayectoria simulada se ejecuta:

```bash
python3 main.py
```

El resultado es una serie de pares `distancia, velocidad`.

Para generar gráficas de la marcha tipo con varias estaciones (requiere `matplotlib`):

```bash
python3 main.py --plot
```

Se mostrarán las curvas de **velocidad contra distancia** y **velocidad contra tiempo** para un recorrido con paradas de 30 segundos en cada estación.
