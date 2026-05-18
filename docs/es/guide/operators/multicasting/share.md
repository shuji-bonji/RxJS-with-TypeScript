---
description: "Describe cómo implementar la multidifusión utilizando el operador share(). Comparte el mismo Observable con varios suscriptores para reducir el procesamiento duplicado (llamadas a la API, cálculos); diferencias con shareReplay(), conversión Cold/Hot e implementación segura de tipos en TypeScript."
---

# share - compartir Observable con múltiples suscriptores

El operador `share()` es el más sencillo para implementar la multidifusión en RxJS.
Al permitir que varios suscriptores compartan la misma fuente de datos, se puede reducir el procesamiento duplicado (solicitudes de API, cálculos, etc.).

[Documentación oficial de RxJS - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Uso básico

```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Resultados de la ejecución

```
Observer 1 Inicio de la suscripción
Fuente.: 0
Observer 1: 0
Fuente.: 1
Observer 1: 1
Observer 2 Inicio de la suscripción
Fuente.: 2
Observer 1: 2
Observer 2: 2
Fuente.: 3
Observer 1: 3
Observer 2: 3
Observer 1 Darse de baja
Fuente.: 4
Observer 2: 4
```

**PUNTO IMPORTANTE**:.
- El procesamiento en origen (`tap`) se ejecuta una sola vez
- Todos los abonados reciben el mismo valor
- Los abonados que se incorporan en mitad del proceso sólo reciben el valor después de incorporarse

## 💡 Cómo funciona share()

share() es un operador de multidifusión estándar de RxJS. Internamente, utiliza Subject para transmitir a múltiples suscriptores.

> [!NOTE]

> **Cambiado desde RxJS v7**: anteriormente descrito como una combinación de `multicast()` y `refCount()`, estos operadores fueron obsoletos en v7 y eliminados en v8. Actualmente, "share()` es el método estándar de multidifusión. Para más información, consulte [Documentación Oficial de RxJS - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Flujo de operación**:.
- **En la primera suscripción**: inicia una conexión con el Observable de origen y crea un Subject interno.
- **Suscriptores añadidos**: compartir la conexión existente (valores de difusión a través de Subject).
- **Todos los suscriptores dados de baja**: desconectar la conexión con el origen (si `resetOnRefCountZero: true`)
- Suscribirse de nuevo**: iniciar una nueva conexión (en función de la configuración de restablecimiento)

## 🎯 Opciones avanzadas de control (RxJS 7+)

En RxJS 7+, el comportamiento puede controlarse con precisión pasando opciones a `share()`.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Fuente.: ${value}`)),
  share({
    resetOnError: true,       // Reiniciar por error
    resetOnComplete: true,     // Restablecer al finalizar
    resetOnRefCountZero: true, // Abonado0Reiniciar cuando
  })
);
```

### Detalles de las opciones

```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Control avanzado con opción de conector

La opción `connector` puede utilizarse para conseguir un comportamiento equivalente al de `shareReplay`.


```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// ReplaySubjectcon el último1Buffer de casos
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Primer abonado
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 2.5Suscribirse después de segundos (pasados1(recibir el último caso)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**Resultados de la ejecución**:.

```
Fuente.: 0
Observer 1: 0
Fuente.: 1
Observer 1: 1
Observer 2: 1  // ← Recibir el último valor, incluso si se unió en el medio
Fuente.: 2
Observer 1: 2
Observer 2: 2
...
```

```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

> Este método puede usarse como alternativa a `shareReplay(1)`. Establecer `resetOnRefCountZero: false` mantendrá viva la conexión incluso si el recuento de referencias llega a cero, evitando el problema de la "caché persistente" de `shareReplay`.

## 📊 Comparación sin share().

### ❌ sin share() (Cold Observable).


```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Fuente.: ${value}`))
);

// Suscriptores1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Suscriptores2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Resultado de la ejecución**:.

```
Fuente.: 0
Observer 1: 0
Fuente.: 1
Observer 1: 1
Fuente.: 0    // ← Se inicia un nuevo flujo
Observer 2: 0
Fuente.: 2
Observer 1: 2
Fuente.: 1
Observer 2: 1
Fuente.: 2
Observer 2: 2
```

Cada abonado tiene un flujo independiente y el proceso de origen se ejecuta por duplicado.

### ✅ Uso de share() (Observable en caliente).

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Fuente.: ${value}`)),
  share()
);

// Suscriptores1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Suscriptores2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Resultados de la ejecución**:.

```
Fuente.: 0
Observer 1: 0
Fuente.: 1
Observer 1: 1
Observer 2: 1  // ← Compartir el mismo flujo
Fuente.: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Caso práctico.

### Evitar la duplicación de peticiones API.

```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Intercambio periódico de recuperación de datos.


```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

## ⚠️ Notas.

1.**Tiempo de las notas**: los suscriptores que se incorporen a mitad de camino no recibirán los valores anteriores.
2.**Propagación de errores**: si se produce un error, afecta a todos los suscriptores
3.**Gestión de memoria**: no darse de baja correctamente puede provocar fugas de memoria

## 🔄 Operadores relacionados.

- **[shareReplay()](/es/guide/operators/multicasting/shareReplay)**: almacena en búfer valores pasados para suscriptores posteriores.
- **[Subject](/es/guide/subjects/what-is-subject)** - la clase subyacente para la multidifusión.


```typescript
import { interval, share, take, tap } from 'rxjs';

// Contar a intervalosObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente.: ${value}`)),
  share() // Activación de la multidifusión
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5Abonados segundos después1Darse de baja
  setTimeout(() => {
    console.log('Observer 1 Darse de baja');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

> **Operadores obsoletos**: las antiguas APIs de multicast como `publish()`, `multicast()` y `refCount()` fueron obsoletas en RxJS v7 y eliminadas en v8. Utilice `share()` o `connectable()`/`connect()` en su lugar.

## Resumen.

El operador `share()` es,
- Comparte el mismo Observable con múltiples suscriptores
- Evita la ejecución duplicada de peticiones API y el procesamiento pesado
- Conceptos básicos de multidifusión fáciles de usar
- Opciones de control detalladas disponibles en RxJS 7+.

Cuando varios componentes necesitan la misma fuente de datos, `share()` puede mejorar significativamente el rendimiento.
