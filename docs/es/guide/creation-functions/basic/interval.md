---
description: "interval() - Función de Creación que emite valores continuamente a intervalos especificados: Esencial para polling, tareas periódicas, temporizadores de cuenta regresiva y actualizaciones en tiempo real"
titleTemplate: ':title | RxJS'
---

# interval() - Emisión Continua a Intervalos Especificados

`interval()` es una Función de Creación que emite valores continuamente a intervalos de tiempo especificados.

## Resumen

`interval()` emite continuamente números consecutivos comenzando desde 0 a intervalos de milisegundos especificados. Se usa frecuentemente para procesos de polling y ejecución de tareas periódicas.

**Firma**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Documentación Oficial**: [📘 RxJS Oficial: interval()](https://rxjs.dev/api/index/function/interval)

## Uso Básico

`interval()` emite números que cuentan hacia arriba a un intervalo especificado.

```typescript
import { interval } from 'rxjs';

// Emitir valores cada 1 segundo
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Valor:', value);
});

// Salida (cada 1 segundo):
// Valor: 0
// Valor: 1
// Valor: 2
// Valor: 3
// ... (continúa infinitamente)
```

## Características Importantes

### 1. Números Consecutivos Comenzando desde 0

`interval()` siempre emite enteros que comienzan en 0 e incrementan en 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Obtener solo los primeros 5 valores
).subscribe(value => console.log(value));

// Salida (cada 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Nunca se Completa (Flujo Infinito)

`interval()` no se completa automáticamente y **debe desuscribirse**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Valor:', value);
});

// Desuscribirse después de 5 segundos
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Detenido');
}, 5000);
```

> [!WARNING]
> **Olvidar Desuscribirse Causa Fugas de Memoria**
>
> Debido a que `interval()` continúa emitiendo valores indefinidamente, olvidar desuscribirse puede causar fugas de memoria y problemas de rendimiento. Asegúrate de llamar `unsubscribe()` o usa operadores como `take()`, `takeUntil()` o `takeWhile()` para completar automáticamente.

### 3. Cold Observable

`interval()` es un Cold Observable, que crea un temporizador independiente para cada suscripción.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Suscripción 1
interval$.subscribe(value => console.log('Observador 1:', value));

// Añadir suscripción 2 después de 2 segundos
setTimeout(() => {
  interval$.subscribe(value => console.log('Observador 2:', value));
}, 2000);

// Salida:
// Observador 1: 0
// Observador 1: 1
// Observador 2: 0  ← Comienza desde 0 con temporizador independiente
// Observador 1: 2
// Observador 2: 1
```

> [!NOTE]
> **Características de Cold Observable**:
> - Se inicia una ejecución independiente para cada suscripción
> - Cada suscriptor recibe su propio flujo de datos
> - Se inicia un temporizador independiente para cada suscripción; usa `share()` si necesitas compartir datos
>
> Ver [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables) para más información.

## Diferencia Entre interval() y timer()

Aunque `interval()` y `timer()` son similares, hay algunas diferencias importantes.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - comienza inmediatamente, emisión continua
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - comienza después de un retraso
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Salida:
// interval: 0  (después de 1 segundo)
// interval: 1  (después de 2 segundos)
// timer: 0     (después de 2 segundos)
// interval: 2  (después de 3 segundos)
// timer: 1     (después de 3 segundos)
// timer: 2     (después de 4 segundos)
```

| Función de Creación | Momento de Inicio | Propósito |
|-------------------|--------------|---------|
| `interval(1000)` | Comienza inmediatamente (primer valor después de 1 segundo) | Ejecución periódica |
| `timer(2000, 1000)` | Comienza después del tiempo especificado | Ejecución periódica con retraso |
| `timer(2000)` | Emite solo una vez después del tiempo especificado | Ejecución retrasada |

## Casos de Uso Prácticos

### 1. Polling de API

Llamar API a intervalos regulares para actualizar datos.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Hacer polling de API cada 5 segundos
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError((error: unknown) => {
    console.error('Error de API:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Actualización de estado:', data);
});

// Detener según sea necesario
// subscription.unsubscribe();
```

### 2. Temporizador de Cuenta Regresiva

Implementar una cuenta regresiva para el límite de tiempo.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Cuenta regresiva desde 10 segundos
  takeWhile(time => time >= 0) // Auto-completar en 0
);

countdown$.subscribe({
  next: time => console.log(`Tiempo restante: ${time} segundos`),
  complete: () => console.log('¡Se acabó el tiempo!')
});

// Salida (cada 1 segundo):
// Tiempo restante: 10 segundos
// Tiempo restante: 9 segundos
// ...
// Tiempo restante: 0 segundos
// ¡Se acabó el tiempo!
```

### 3. Función de Autoguardado

Autoguardar contenidos de formulario periódicamente.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Crear formulario
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Ingresa texto';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Autoguardar 3 segundos después de que la entrada se detenga (acortado para demo)
input$.pipe(
  debounceTime(3000), // Si no hay entrada durante 3 segundos
  switchMap(() => {
    const formData = new FormData(form);
    // Demo: Simular con Promise en lugar de API real
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Autoguardado:', result);
});
```

### 4. Visualización de Reloj en Tiempo Real

Actualizar la hora actual en tiempo real.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Crear elemento para visualización del reloj
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Salida: La hora actual se actualiza cada segundo
```

## Uso en Pipeline

`interval()` se usa como punto de partida para pipelines o como disparador de control de tiempo.

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// Contar solo segundos pares
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `Suma de pares: ${sum}`)
).subscribe(console.log);

// Salida (cada 1 segundo):
// Suma de pares: 0
// Suma de pares: 2  (0 + 2)
// Suma de pares: 6  (0 + 2 + 4)
// Suma de pares: 12 (0 + 2 + 4 + 6)
```

## Errores Comunes

### 1. Olvidar Desuscribirse

```typescript
// ❌ Incorrecto - se ejecuta infinitamente sin desuscripción
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Valor:', value); // Se ejecuta para siempre
  });
}

startPolling();

// ✅ Correcto - mantener suscripción y desuscribirse según sea necesario
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Valor:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// Llamar stopPolling() según sea necesario
```

### 2. Múltiples Suscripciones Crean Temporizadores Independientes

```typescript
// ❌ No intencionado - se crean dos temporizadores independientes
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observador 1:', value));
interval$.subscribe(value => console.log('Observador 2:', value));
// Dos temporizadores se ejecutan en paralelo

// ✅ Correcto - compartir un temporizador
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observador 1:', value));
interval$.subscribe(value => console.log('Observador 2:', value));
// Se comparte un temporizador
```

## Consideraciones de Rendimiento

Aunque `interval()` es ligero, el rendimiento debe considerarse al ejecutar a intervalos cortos.

> [!TIP]
> **Consejos de Optimización**:
> - No realizar procesamiento innecesario (refinar con `filter()`)
> - Usar intervalos cortos (menos de 100ms) con precaución
> - Asegurar que las suscripciones se desuscriban
> - Si se necesitan múltiples Observers, compartirlos con `share()`

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Problema de rendimiento - procesamiento pesado cada 100ms
interval(100).subscribe(() => {
  // Procesamiento pesado
  heavyCalculation();
});

// ✅ Optimización - procesar solo cuando sea necesario
interval(100).pipe(
  filter(count => count % 10 === 0), // Una vez por segundo (una vez cada 10 veces)
  share() // Compartir entre múltiples Observers
).subscribe(() => {
  heavyCalculation();
});
```

## Funciones de Creación Relacionadas

| Función | Diferencia | Uso |
|----------|------|----------|
| **[timer()](/es/guide/creation-functions/basic/timer)** | Comienza después de retraso, o emite solo una vez | Ejecución retrasada o procesamiento único |
| **[fromEvent()](/es/guide/creation-functions/basic/fromEvent)** | Dirigido por eventos | Procesamiento según operaciones del usuario |
| **range()** | Emite números en rango especificado inmediatamente | Cuando no se necesita control de tiempo |

## Resumen

- `interval()` emite valores continuamente a intervalos especificados
- Emite enteros consecutivos comenzando desde 0
- No se auto-completa, debe desuscribirse
- Funciona como Cold Observable (temporizador independiente para cada suscripción)
- Ideal para polling, ejecución periódica, cuenta regresiva, etc.

## Próximos Pasos

- [timer() - Comenzar a Emitir Después de un Retraso](/es/guide/creation-functions/basic/timer)
- [fromEvent() - Convertir Eventos a Observable](/es/guide/creation-functions/basic/fromEvent)
- [Volver a Funciones de Creación Básicas](/es/guide/creation-functions/basic/)
