---
description: Este curso explica la clasificación de tareas (síncronas, microtareas y macrotareas) y su relación con cada scheduler en RxJS desde los fundamentos, incluyendo cómo funciona el event loop de JavaScript, la diferencia en el orden de ejecución, y la implementación y operación de setTimeout, Promise, queueMicrotask, etc. y adquirir conocimiento que puede usarse para seleccionar un scheduler para RxJS.
---

# Conocimientos Básicos de Tareas y Schedulers

## ¿Qué es el Procesamiento Síncrono?
El procesamiento síncrono se ejecuta inmediatamente en el orden en que está escrito el código, y no procede al siguiente proceso hasta que el proceso anterior se haya completado.

#### Ejemplo
```ts
console.log('A');
console.log('B');
console.log('C');

// Salida:
// A
// B
// C
```


## ¿Qué es el Procesamiento Asíncrono?
El procesamiento asíncrono es procesamiento que no se ejecuta inmediatamente, sino que se ejecuta después de que el procesamiento síncrono actual haya terminado.
El procesamiento asíncrono incluye "macrotareas" y "microtareas".


## Macrotarea
- Una tarea que se ejecuta en el siguiente ciclo del event loop.
- Ejemplos: `setTimeout`, `setInterval`, eventos del navegador

#### Ejemplo de Ejecución
```ts
console.log('Inicio');
setTimeout(() => console.log('Macrotarea'), 0);
console.log('Fin');

// Salida:
// Inicio
// Fin
// Macrotarea
```

### Soporte de RxJS
- `asyncScheduler`
  - Usa `setTimeout` internamente
  - Funciona como una macrotarea

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hola')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Salida:
// Hola
```


## Microtarea
- Una tarea que se ejecuta inmediatamente después de que la tarea actual termina, pero antes de que comience la siguiente tarea.
- Ejemplos: `Promise.then`, `queueMicrotask`

#### Ejemplo de Ejecución
```ts
console.log('Inicio');
Promise.resolve().then(() => console.log('Microtarea'));
console.log('Fin');

// Salida:
// Inicio
// Fin
// Microtarea
```

### Soporte de RxJS
- `asapScheduler`
  - Usa `Promise.resolve().then()` internamente
  - Funciona como una microtarea

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hola')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Salida:
// Hola
```


## Tarea Síncrona
- Código normal a ejecutar inmediatamente.

### Soporte de RxJS
- `queueScheduler`
  - Parece ser síncrono, pero permite un control fino a través de la puesta en cola de tareas.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Ahora')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Salida:
// Ahora
```


## Resumen del Orden de Ejecución

#### Ejemplo de Código
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Salida:
// 1
// 4
// 3 (Promise) 👈 Microtarea
// 2 (setTimeout) 👈 Macrotarea
```


## Tabla de Correspondencia entre Tareas y Schedulers de RxJS

| Tipo         | Ejemplo                       | Scheduler de RxJS  |
|--------------|-------------------------------|---------------------|
| Síncrono     | Código normal                 | `queueScheduler`    |
| Microtarea   | Promise.then, queueMicrotask  | `asapScheduler`     |
| Macrotarea   | setTimeout, setInterval       | `asyncScheduler`    |
