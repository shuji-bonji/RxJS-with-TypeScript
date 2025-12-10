---
description: Describe cómo implementar multicasting usando el operador share(). Describe cómo compartir el mismo Observable con múltiples suscriptores para reducir el procesamiento duplicado y proporciona opciones de control detalladas.
titleTemplate: ':title | RxJS'
---

# share - Compartir un Observable con Múltiples Suscriptores

El operador `share()` es el operador de multicasting más fácil de implementar en RxJS.
Múltiples suscriptores pueden compartir la misma fuente de datos para reducir el procesamiento duplicado (solicitudes API, procesamiento de cálculo, etc.).

[📘 RxJS Official Documentation - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Uso Básico

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable contando en intervalos
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share() // Habilitar multicasting
);

// Primer suscriptor
console.log('Suscripción del Observador 1 iniciada');
const subscription1 = source$.subscribe(value =>
  console.log(`Observador 1: ${value}`)
);

// Añadir segundo suscriptor después de 2.5 segundos
setTimeout(() => {
  console.log('Suscripción del Observador 2 iniciada');
  source$.subscribe(value =>
    console.log(`Observador 2: ${value}`)
  );

  // Cancelar suscripción del suscriptor 1 después de 2.5 segundos
  setTimeout(() => {
    console.log('Observador 1 desuscrito');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Resultado de Ejecución

```
Suscripción del Observador 1 iniciada
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Suscripción del Observador 2 iniciada
Source: 2
Observador 1: 2
Observador 2: 2
Source: 3
Observador 1: 3
Observador 2: 3
Observador 1 desuscrito
Source: 4
Observador 2: 4
```

**Puntos Importantes**:
- El procesamiento de la fuente (`tap`) se ejecuta solo una vez
- Todos los suscriptores reciben el mismo valor
- Los suscriptores que se unen en el medio solo recibirán valores después de unirse

## 💡 Cómo Funciona share()

`share()` es un operador estándar de multicasting de RxJS. Internamente, usa Subject para transmitir a múltiples suscriptores.

> [!NOTE]
> **Cambios en RxJS v7 y posteriores**: Anteriormente se explicaba como una combinación de `multicast()` y `refCount()`, estos operadores fueron deprecados en v7 y eliminados en v8. Actualmente, `share()` es el método estándar de multicasting. Para más detalles, consulta [RxJS Official Documentation - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Flujo de Operación**:
- **En la primera suscripción**: Inicia una conexión al Observable fuente y crea un Subject interno
- **Añadir suscriptores**: Comparte la conexión existente (transmite valores a través del Subject)
- **Todos los suscriptores desuscritos**: Desconecta de la fuente (si `resetOnRefCountZero: true`)
- **Resuscribir**: Comienza como una nueva conexión (dependiendo de la configuración de reinicio)

## 🎯 Opciones de Control Avanzadas (RxJS 7+)

En RxJS 7 y posteriores, puedes pasar opciones a `share()` para controlar finamente su comportamiento.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source: ${value}`)),
  share({
    resetOnError: true,       // Reiniciar en error
    resetOnComplete: true,     // Reiniciar al completar
    resetOnRefCountZero: true, // Reiniciar cuando el conteo de suscriptores llega a cero
  })
);
```

### Opciones en Detalle

| Opción | Predeterminado | Descripción |
|-----------|----------|------|
| `resetOnError` | `true` | Reinicia el estado interno en error |
| `resetOnComplete` | `true` | Reinicia el estado interno al completar el flujo |
| `resetOnRefCountZero` | `true` | Desconecta cuando el conteo de suscriptores llega a cero |
| `connector` | `() => new Subject()` | Especifica Subject personalizado |

### Control Avanzado Usando la Opción connector

Usando la opción `connector`, puedes lograr un comportamiento equivalente a `shareReplay`.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Almacenar en búfer el último 1 elemento usando ReplaySubject
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Primer suscriptor
source$.subscribe(value => console.log(`Observador 1: ${value}`));

// Suscribirse después de 2.5 segundos (recibe el último 1 elemento del pasado)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 2500);
```

**Resultado de Ejecución**:
```
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Observador 2: 1  // ← Recibe valor anterior incluso al unirse a mitad de camino
Source: 2
Observador 1: 2
Observador 2: 2
...
```

> [!TIP]
> Este método se puede usar como alternativa a `shareReplay(1)`. Al establecer `resetOnRefCountZero: false`, puedes mantener la conexión incluso cuando el conteo de referencias llega a cero, evitando el problema de "caché persistente" de `shareReplay`.

## 📊 Comparación con y sin share()

### ❌ Sin share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`))
);

// Suscriptor 1
source$.subscribe(value => console.log(`Observador 1: ${value}`));

// Suscriptor 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 1500);
```

**Resultado de Ejecución**:
```
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Source: 0    // ← Nuevo flujo comienza
Observador 2: 0
Source: 2
Observador 1: 2
Source: 1
Observador 2: 1
Source: 2
Observador 2: 2
```

Cada suscriptor tiene un flujo independiente, y el procesamiento de la fuente se ejecuta redundantemente.

### ✅ Con share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  share()
);

// Suscriptor 1
source$.subscribe(value => console.log(`Observador 1: ${value}`));

// Suscriptor 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 1500);
```

**Resultado de Ejecución**:
```
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Observador 2: 1  // ← Comparte el mismo flujo
Source: 2
Observador 1: 2
Observador 2: 2
```

## 💼 Casos de Uso Prácticos

### Prevención de Solicitudes API Duplicadas

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable para obtener información de usuario
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('Solicitud API ejecutada')),
  share() // Prevenir solicitudes duplicadas en múltiples componentes
);

// Componente 1
getUser$.subscribe(user => console.log('Componente 1:', user));

// Componente 2 (solicita casi simultáneamente)
getUser$.subscribe(user => console.log('Componente 2:', user));

// Resultado: La solicitud API se ejecuta solo una vez
```

### Compartir Recuperación Periódica de Datos

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Obtener lista TODO cada 5 segundos (compartir solicitud API)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('Solicitud API ejecutada')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // Compartir solicitud API entre múltiples suscriptores
);

// Usar el mismo flujo de datos en múltiples componentes
sharedTodos$.subscribe(todos => console.log('Componente A:', todos));
sharedTodos$.subscribe(todos => console.log('Componente B:', todos));

// Resultado: La solicitud API se ejecuta solo una vez cada 5 segundos, ambos componentes reciben los mismos datos
```

## ⚠️ Notas Importantes

1. **Ten cuidado con el timing**: Los suscriptores que se unen a mitad de camino no pueden recibir valores pasados
2. **Propagación de errores**: Cuando ocurre un error, todos los suscriptores se ven afectados
3. **Gestión de memoria**: No desuscribirse adecuadamente puede causar fugas de memoria

## 🔄 Operadores Relacionados

- **[shareReplay()](/es/guide/operators/multicasting/shareReplay)** - Almacena en búfer valores pasados y los proporciona a suscriptores posteriores
- **[Subject](/es/guide/subjects/what-is-subject)** - La clase que forma la base del multicasting

> [!WARNING]
> **Operadores deprecados**: Las APIs antiguas de multicasting como `publish()`, `multicast()`, `refCount()` fueron deprecadas en RxJS v7 y eliminadas en v8. Usa `share()` o `connectable()`/`connect()` en su lugar.

## Resumen

El operador `share()`:
- Comparte el mismo Observable entre múltiples suscriptores
- Previene la ejecución duplicada de solicitudes API y procesamiento pesado
- Conceptos básicos de multicasting fáciles de usar
- Opciones de control fino disponibles en RxJS 7+

Cuando múltiples componentes necesitan la misma fuente de datos, usar `share()` puede mejorar significativamente el rendimiento.
