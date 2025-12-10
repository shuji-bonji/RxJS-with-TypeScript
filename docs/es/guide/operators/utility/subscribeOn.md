---
description: El operador subscribeOn controla cuándo comenzar a suscribirse a Observable con el scheduler especificado y cambia el contexto de ejecución de todo el stream.
---

# subscribeOn - Controlar Cuándo Comenzar a Suscribirse

El operador `subscribeOn` controla el **momento de inicio de suscripción de Observable y el contexto de ejecución con el scheduler especificado**. Afecta el momento de ejecución de todo el stream.

## 🔰 Sintaxis Básica y Operación

Asincroniza el inicio de una suscripción especificando un scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Inicio');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Valor:', v));

console.log('Fin');

// Salida:
// Inicio
// Fin
// Valor: 1
// Valor: 2
// Valor: 3
```

El inicio de la suscripción en sí se asincroniza, por lo que la llamada a `subscribe()` devuelve inmediatamente.

[🌐 Documentación Oficial de RxJS - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Ejemplos de Uso Típicos

- **Asincronizar procesos de inicialización pesados**: Retrasar el inicio de carga de datos, etc.
- **Prevenir congelamiento de UI**: Iniciar suscripciones asincrónicamente para mantener la capacidad de respuesta
- **Priorizar procesamiento**: Controlar momento de inicio de múltiples streams
- **Control de temporización en pruebas**: Controlar usando TestScheduler

## 🧪 Ejemplo de Código Práctico 1: Asincronizar Procesamiento de Inicialización Pesado

Este es un ejemplo de iniciar lectura de datos e inicialización de forma asíncrona.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Procesamiento de inicialización pesado';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Simular procesamiento de inicialización pesado
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Carga de datos iniciada...', '#fff9c4');

  // Simular procesamiento pesado
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Carga de datos completada', '#c8e6c9');
  subscriber.next(`Resultado: ${sum}`);
  subscriber.complete();
});

addLog('Inicio de suscripción (UI operable)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Asincronizar inicio de suscripción
  )
  .subscribe({
    next: result => addLog(`Recibido: ${result}`, '#c8e6c9'),
    complete: () => addLog('Completado', '#e3f2fd')
  });

addLog('Después de solicitud de suscripción (la ejecución continúa inmediatamente)', '#e3f2fd');
```

- El inicio de la suscripción es asíncrono, la UI responde inmediatamente
- El procesamiento pesado se realiza de forma asíncrona
- El hilo principal no está bloqueado

## 🧪 Ejemplo de Código Práctico 2: Control de Prioridad de Múltiples Streams

Este es un ejemplo de controlar el momento de inicio de múltiples streams.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Control de prioridad';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Inicio', '#e3f2fd');

// Tarea de alta prioridad (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`Alta prioridad: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Tarea de prioridad normal (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Prioridad normal: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Solicitud de suscripción completada', '#e3f2fd');
```

- Diferentes schedulers controlan prioridades
- `asapScheduler` inicia la ejecución antes que `asyncScheduler`

## 🆚 Diferencias con observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// Ejemplo de observeOn
console.log('=== observeOn ===');
console.log('1: Inicio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (sinc)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (async)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: Fin');

// Ejemplo de subscribeOn
console.log('\n=== subscribeOn ===');
console.log('1: Inicio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: Fin');
```

**Principales diferencias**:

| Elemento | observeOn | subscribeOn |
|:---|:---|:---|
| **Alcance de Efectos** | Solo procesamiento subsiguiente | Stream completo |
| **Objetivo de Control** | Momento de publicación del valor | Momento de inicio de suscripción |
| **Posicionamiento** | Importante (el comportamiento cambia según dónde lo coloque) | Igual donde sea que lo coloque |
| **Uso Múltiple** | Se aplica el último | Se aplica el primero |

> [!NOTE]
> Para más información sobre `observeOn`, consulte [observeOn](./observeOn.md).

## ⚠️ Notas Importantes

### 1. La Posición de Colocación No Tiene Efecto

`subscribeOn` tiene el mismo efecto sin importar dónde lo coloque en el pipeline.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Patrón 1: Primero
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Patrón 2: Último
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Ambos funcionan igual
```

### 2. Múltiples subscribeOn Aplican el Primero

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // Este se usa
    subscribeOn(asapScheduler)    // Este se ignora
  )
  .subscribe();
```

Se usa el scheduler del primer `subscribeOn` (`asyncScheduler`).

### 3. Algunos Observables No Tienen Efecto

Los Observables con su propio scheduler, como `interval` y `timer`, no se ven afectados por `subscribeOn`.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn no tiene efecto
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval usa su propio scheduler
  )
  .subscribe();

// ✅ Especificar scheduler en argumento de interval
interval(1000, asyncScheduler)
  .subscribe();
```

## Ejemplos de Combinación Práctica

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Inicio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap 1 (async)')),
    subscribeOn(asyncScheduler),        // Asincronizar inicio de suscripción
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Sincronizar emisión de valor con fotograma de animación
    tap(() => console.log('Tap 2 (fotograma de animación)'))
  )
  .subscribe(v => console.log('Valor:', v));

console.log('Fin');

// Orden de ejecución:
// Inicio
// Fin
// Tap 1 (async)
// Tap 1 (async)
// Tap 1 (async)
// Tap 2 (fotograma de animación)
// Valor: 2
// ... (continúa abajo)
```

## Directrices de Uso

### Caso 1: Desea Retrasar el Inicio de las Suscripciones
```ts
// → usar subscribeOn
of(data)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Caso 2: Quiero Hacer Asíncrono un Proceso Específico
```ts
// → usar observeOn
of(data)
  .pipe(
    map(procesamiento pesado),
    observeOn(asyncScheduler),  // Asincronizar solo después del procesamiento pesado
    map(procesamiento ligero)
  )
  .subscribe();
```

### Caso 3: Asincronizar Todo el Proceso + Controlar Adicionalmente una Parte
```ts
// → usar subscribeOn + observeOn juntos
of(data)
  .pipe(
    subscribeOn(asyncScheduler),           // Asincronizar todo el proceso
    map(procesamiento 1),
    observeOn(animationFrameScheduler),    // Cambiar para animación
    map(procesamiento 2)
  )
  .subscribe();
```

## 📚 Operadores Relacionados

- **[observeOn](./observeOn)** - Controla cuándo se emiten valores
- **[delay](./delay)** - Retardo de tiempo fijo

## 📖 Documentos Relacionados

- **[Control de Procesamiento Asíncrono](/es/guide/schedulers/async-control.md)** - Fundamentos de Scheduler
- **[Tipos y Uso de Schedulers](/es/guide/schedulers/types.md)** - Detalles de cada scheduler

## ✅ Resumen

El operador `subscribeOn` controla el momento y el contexto de ejecución para el inicio de la suscripción.

- ✅ Asincroniza el inicio de suscripción para todo el stream
- ✅ Útil para asincronizar procesos de inicialización pesados
- ✅ Útil para prevenir congelamientos de UI
- ✅ La posición de colocación no tiene efecto
- ⚠️ Cuando se usan múltiples Observables, se aplica el primero
- ⚠️ No efectivo para algunos Observables
- ⚠️ Propósito diferente de `observeOn`
