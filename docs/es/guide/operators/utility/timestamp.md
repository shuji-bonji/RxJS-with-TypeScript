---
description: El operador timestamp agrega una marca de tiempo a cada valor y registra el momento en que se emitió el valor, que se puede usar para medición de rendimiento y depuración.
---

# timestamp - Agregar Marca de Tiempo

El operador `timestamp` **agrega una marca de tiempo** a cada valor en el stream. Esto se puede usar para medición de rendimiento, depuración y análisis de series temporales de eventos registrando el momento exacto en que se emitió el valor.

## 🔰 Sintaxis Básica y Operación

Convierte cada valor en un objeto con marca de tiempo.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Salida:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

El objeto devuelto tiene la siguiente estructura:
- `value`: El valor original
- `timestamp`: Marca de tiempo (tiempo Unix en milisegundos)

[🌐 Documentación Oficial de RxJS - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Ejemplos de Uso Típicos

- **Medición de rendimiento**: Medir tiempo de procesamiento
- **Análisis de temporización de eventos**: Medir intervalos entre acciones del usuario
- **Depuración y logging**: Registrar el momento de emisión del valor
- **Registro de datos de series temporales**: Almacenamiento con marca de tiempo de datos de sensores, etc.

## 🧪 Ejemplo de Código Práctico 1: Medición de Intervalos de Clic

Este es un ejemplo de medir el intervalo de clic del usuario.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Medición de intervalo de clic';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Por favor haga clic';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // Mostrar más reciente en la parte superior
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('es-ES')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? '¡Clic rápido!' :
                  data.interval < 1000 ? 'Normal' : 'Lento';

    addLog(
      `${data.clickNumber}º clic: ${data.interval}ms de intervalo [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Por favor haga clic en el botón (intervalo medido desde el 2º clic)', '#e3f2fd');
```

- Medir con precisión el intervalo de clic
- Visualización codificada por colores según la velocidad
- Registra el momento de ocurrencia con marca de tiempo

## 🧪 Ejemplo de Código Práctico 2: Medición de Tiempo de Procesamiento

Este es un ejemplo de medir el tiempo tomado para cada proceso.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Medición de tiempo de procesamiento';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Procesamiento iniciado...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Marca de tiempo antes del procesamiento
    map(data => {
      const start = data.timestamp;

      // Simular procesamiento pesado (tiempo de procesamiento aleatorio)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('es-ES', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Valor${result.value}: inicio=${result.startTime}, tiempo de procesamiento=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Todo el procesamiento completado ---');
    }
  });
```

- Registrar el momento de inicio de cada valor
- Medir el tiempo tomado para el procesamiento
- Usar para análisis de rendimiento

## Uso de Marcas de Tiempo

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Procesamiento usando marca de tiempo
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Salida:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Notas Importantes

### 1. Precisión de Marca de Tiempo

Debido a que se usa `Date.now()` de JavaScript, la precisión es en milisegundos.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Eventos de alta frecuencia (intervalo de 1ms)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Valor: ${data.value}, Marca de tiempo: ${data.timestamp}`);
  });
// Puede tener la misma marca de tiempo
```

Si necesita mayor precisión, considere usar `performance.now()`.

### 2. La Marca de Tiempo es en el Momento de Emisión

La marca de tiempo es el momento en que se emitió el valor, no cuando se generó.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // Retraso de 1 segundo
    timestamp()       // Marca de tiempo después del retraso
  )
  .subscribe(console.log);
```

### 3. Cambio de Estructura de Objeto

Usar `timestamp` envuelve el valor en un objeto.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Acceder al valor original con .value
  )
  .subscribe(console.log);
// Salida: 2, 4, 6
```

## 📚 Operadores Relacionados

- **[tap](./tap)** - Realizar efectos secundarios (para depuración)
- **[delay](./delay)** - Retardo de tiempo fijo
- **[timeout](./timeout)** - Control de tiempo de espera

## ✅ Resumen

El operador `timestamp` da una marca de tiempo para cada valor.

- ✅ Registra con precisión el momento en que se emite cada valor
- ✅ Útil para medición de rendimiento
- ✅ Permite análisis de intervalos de eventos
- ✅ Útil para depuración y logging
- ⚠️ Precisión en milisegundos
- ⚠️ Los valores se envuelven en objetos
- ⚠️ Las marcas de tiempo son en el momento de emisión
