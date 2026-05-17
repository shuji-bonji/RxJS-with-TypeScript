---
description: "El operador withLatestFrom combina el último valor de otro stream cada vez que el Observable principal emite: Ideal para validación de formularios y sincronización de estado"
titleTemplate: ':title'
---

# withLatestFrom - Con ultimo valor

El operador `withLatestFrom` **cada vez que se emite un valor en el stream principal**,
combina el **último valor** de otro stream y lo emite.


## 🔰 Sintaxis Básica y Uso

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Contador al momento del clic: ${timerValue}`)
  )
  .subscribe(console.log);

// Salida:
// Contador al momento del clic: 1
// Contador al momento del clic: 2
// Contador al momento del clic: 2
// Contador al momento del clic: 5

```

- El Observable principal (en este caso, clics) actúa como el disparador,
- El **último valor** del Observable secundario (en este caso, contador) se combina y emite cada vez.

[🌐 Documentación Oficial de RxJS - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)

> [!WARNING] Atención en código de producción
> El ejemplo anterior omite la cancelación de suscripción de `fromEvent` para simplificar la explicación. En código real, gestione explícitamente el ciclo de vida con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Detalles: [Superar dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Patrones de Uso Típicos

- **Obtener último estado al momento de acción del usuario**
- **Referenciar datos en caché al momento de solicitud**
- **Vinculación de datos activada por evento**


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de recuperación y visualización del último valor de un campo de entrada cada 2 segundos.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom: Obtener Última Entrada Cada 2 Segundos:';
document.body.appendChild(title);

// Crear campo de entrada
const nameInput = document.createElement('input');
nameInput.placeholder = 'Ingrese nombre';
document.body.appendChild(nameInput);

// Crear área de salida
const output = document.createElement('div');
document.body.appendChild(output);

// Observable de entrada
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Comenzar con cadena vacía
);

// Temporizador (se dispara cada 2 segundos)
const timer$ = interval(2000);

// Obtener último valor de entrada cada vez que se dispara el temporizador
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `Recuperación de 2 segundos: Nombre: ${name}`;
  output.prepend(item);
});

```

- Mientras el usuario continúa escribiendo,
- **La última entrada se recupera y muestra** cada 2 segundos.
