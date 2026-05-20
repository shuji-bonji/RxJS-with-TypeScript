---
description: "El operador sampleTime es un operador de filtrado RxJS que muestrea periódicamente los últimos valores del flujo en intervalos de tiempo especificados. Es ideal para tomar instantáneas periódicas."
---

# sampleTime - obtener periódicamente el último valor

El operador `sampleTime` **muestrea periódicamente** el último valor del Observable fuente en **intervalos de tiempo especificados** y lo muestra.
Al igual que una instantánea periódica, recupera el valor más reciente en ese momento.

## 🔰 Sintaxis básica y uso

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Muestras segundo a segundo');
});
```

**Flujo de operación**:.
1. temporizador dispara periódicamente cada 2 segundos.
2. salida si hay un evento de clic reciente en ese momento
3. si no hay ningún valor durante el periodo de muestreo, no hay salida

> [!WARNING] 本番コードでの注意

> El ejemplo anterior omite la desuscripción de `fromEvent` para simplificar la explicación. En código real, utilice `takeUntil(destroy$)`, `take(N)` o `Subscription.unsubscribe()` para gestionar explícitamente el ciclo de vida. Más información: [Superando dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)

[Documentación oficial de RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Patrones de utilización típicos

- **Adquisición recurrente de datos de sensores**: información actualizada de temperatura y ubicación cada segundo.
- **Panel de control en tiempo real**: actualizaciones periódicas del estado.
- **Supervisión del rendimiento**: recopilación de métricas a intervalos regulares.
- **Procesamiento de fotogramas de juego**: muestreo periódico para control de FPS

## 🧠 Ejemplo práctico de código 1: muestreo periódico de la posición del ratón

Este es un ejemplo de muestreo de la posición del ratón cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreación
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Muestreo de la posición del ratón (1(cada segundo)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Mover el ratón dentro de esta área';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Evento de movimiento del ratón
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Muestreo cada segundo
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Muestreo #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Posición: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Máx.10Visualización de hasta
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Si el ratón se mueve continuamente, sólo la última posición actual se muestrea cada segundo.
- Si el ratón no se mueve durante un segundo, nada se emite para ese período.

## 🎯 Ejemplo práctico de código 2: panel de datos en tiempo real

Este ejemplo muestra cómo los datos del sensor se pueden muestrear periódicamente y se muestran en un tablero de instrumentos.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreación
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Cuadro de mandos de supervisión de sensores';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Creación de tarjeta de cuadro de mando
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Temperatura', '°C');
const humidityValue = createCard('Humedad', '%');
const pressureValue = createCard('Presión barométrica', 'hPa');
const lightValue = createCard('Iluminancia', 'lux');

// Flujo de datos del sensor (100msActualizado cada)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Cuadro de mandos muestreado y actualizado cada segundo
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Efecto de animación
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- Los datos del sensor se actualizan cada 100 ms, mientras que el cuadro de mandos se actualiza con valores muestreados cada 2 segundos.
- El rendimiento puede optimizarse mostrando flujos de datos de alta frecuencia a intervalos apropiados.

## 🆚 Comparación con operadores similares

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Muestreo del último valor en ese momento cada segundo
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Ejemplos de salida: 2, 5, 8(1Instantánea cada segundo)

// throttleTime: Después de la salida del primer valor,1Ignorado durante 2 segundos después de la salida del primer valor
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Ejemplos de salida: 0, 3, 6, 9(primer valor de cada periodo)

// auditTime: Se emite el último valor del periodo1segundos después del primer valor, se emite el último valor del período
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Ejemplos de salida: 2, 5, 8(último valor de cada período)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Muestras segundo a segundo');
});
```

**diferencias visuales**:.

Entrada: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Muestreo periódico)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignorado durante el periodo por el principio)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Último valor al final del periodo)
```

## ⚠️ Notas.

### 1. ningún valor durante el período de la muestra

Si no hay nuevos valores en el tiempo de muestreo, no se produce ninguna salida.

```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Muestras tomadas');
});
// 2Durante segundos1No hay salida si no hay pulsaciones
```

### 2. Esperar hasta la primera temporización de muestreo

El `sampleTime` no emitirá nada hasta que haya transcurrido el tiempo especificado.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// Primer valor es1segundos después de la salida del primer valor
```

### 3. completionTime

Cuando una fuente finaliza, la finalización no se propaga hasta la siguiente temporización de muestreo.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Completado')
});
// 1Segundos después: 3
// 1Segundos después: Completado
```

### 4. uso de memoria

La eficiencia de memoria es buena ya que sólo se mantiene internamente un último valor.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Flujo de alta frecuencia (10mspor segundo)
interval(10).pipe(
  sampleTime(1000) // 1Muestreo cada segundo
).subscribe(console.log);
// La memoria sólo retiene los últimos1Sólo se retienen en memoria los dos valores más recientes
```

## 💡 Diferencias con sample

`sample` utiliza otro Observable como disparador, mientras que `sampleTime` utiliza un intervalo de tiempo fijo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Muestras segundo a segundo');
});
```


## 📚 Operadores relacionados.

- **[sample](https://rxjs.dev/api/operators/sample)** - Muestreo de otro Observable como disparador (documentación oficial).
- **[throttleTime](. /throttleTime)** - Obtener el primer valor al inicio del periodo.
- **[auditTime](. /auditTime)** - Obtener el último valor al final del periodo.
- **[debounceTime](. /debounceTime)** - obtener el valor después de la inactividad

## Resumen.

El operador `sampleTime` muestrea periódicamente el último valor en el intervalo de tiempo especificado.

- ✅ Ideal para tomar instantáneas periódicas.
- ✅ Útil para adelgazar flujos de alta frecuencia.
- ✅ Uso eficiente de la memoria (sólo se conserva un último valor)
- ✅ Ideal para cuadros de mando y supervisión
- ⚠️ Si no hay valores disponibles durante el periodo de muestreo, no se emite nada
- ⚠️ Hay un periodo de espera hasta la primera muestra
- ⚠️ La finalización se propaga en la siguiente temporización de muestreo
