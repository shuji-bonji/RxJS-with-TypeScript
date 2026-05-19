---
description: "El operador filter ordena los valores de un flujo basándose en una función condicional especificada, permitiendo que sólo pasen los valores que satisfacen la condición. Puede ser utilizado como una función Type Guard (predicado de tipo) en TypeScript, y también explica la diferencia entre él y buffer, y las advertencias de hacer de una función predicado una función pura. Esta sección también explica la diferencia entre buffers y funciones puras."
---

# filter - sólo pasar valores que cumplan las condiciones

El operador `filter` ordena los valores de un flujo basándose en una función condicional especificada y sólo permite pasar los valores que cumplan la condición.

## 🔰 Sintaxis básica y uso

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Salidas: 2, 4, 6, 8, 10
```

- Sólo se pasan los valores que coinciden con la condición.
- Funciona de forma similar a `Array.prototype.filter()` en arrays, pero es secuencial en el Observable.

[🌐 Documentación oficial de RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Patrón de utilización típico.

- Validación de valores de entrada de formularios.
- Permitir sólo datos de un tipo o estructura específica
- Filtrado de eventos de sensores y datos de flujo

## 🧠 Ejemplos prácticos de código (con interfaz de usuario)

Listar sólo en tiempo real si el número introducido es par.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Ejemplos prácticos de:';
document.body.appendChild(title);

// Creación de campos de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Introducir valores numéricos';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Crear área de salida
const output = document.createElement('div');
document.body.appendChild(output);

// Flujo de eventos de entrada
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Detección de números pares: ${evenNumber}`;
    output.prepend(item);
  });

```

- Sólo se muestra en la salida si el número es par.
- Se ignoran las entradas impares o no válidas.

> [!WARNING] 本番コードでの注意

> El ejemplo anterior omite la desuscripción de `fromEvent` para simplificar la explicación. En código real, utilice `takeUntil(destroy$)`, `take(N)` o `Subscription.unsubscribe()` para gestionar explícitamente el ciclo de vida. Más información: [Superando dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)

## 🔍 Diferencias con el buffer

| Operador | Operación | Salida. |
|---|---|---|
| filter`. | Descarta los valores que no **cumplen** la condición. | Valores individuales `T`. |
| buffer`. | Almacena** los valores en un array**. | Matriz `T[]`. |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Sólo pasan los valores que cumplen las condiciones
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Salidas: filter: 0
  // Salidas: filter: 2
  // Salidas: filter: 4
});

// buffer - Almacena los valores como una matriz
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Salidas: buffer: [0, 1]
  // Salidas: buffer: [2, 3, 4]
});
```

## ⚠️ Notas.

### 1. las funciones de predicado deben ser funciones puras

Las funciones de predicado con efectos secundarios pueden causar un comportamiento inesperado cuando se resuscribe el flujo.

```ts
// ❌ Mal ejemplo: Efectos secundarios Sí
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Efecto secundario
    return x > 10;
  })
).subscribe();

// ✅ Buen ejemplo: Función pura
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. Usar como función de protección de tipo

Puedes escribirla para devolver un predicado de tipo TypeScript (`x is T`) para restringir el tipo después de pasar `filter`.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Se utiliza como función de protección de tipo
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email no es una función de protección de tipo string Se infiere como un tipo
});
```

> [!TIP] 型ガードの効果

> Al devolver el predicado de tipo `user is User & { email: string }`, `user` después de `filter` hace que `email` sea una propiedad requerida. Llamadas como `user.email.toLowerCase()` pueden ser escritas sin errores de tipo.

## 📚 Operadores relacionados.

- take](/es/guide/operators/filtering/take) - sólo se toman los N primeros valores.
- [first](/es/guide/operators/filtering/first) - se obtiene sólo el primer valor (también puede ser condicional).
- distinct](/es/guide/operators/filtering/distinct) - excluye los valores duplicados
- distinctUntilChanged](/es/guide/operators/filtering/distinctUntilChanged) - excluye el mismo que el último valor

## Resumen.

El operador `filter` es la herramienta de filtrado más básica de RxJS.

- ✅ Sólo se pasan los valores que coinciden con las condiciones.
- ✅ Se puede utilizar de la misma manera que `.filter()` para arrays.
- ✅ También puede usarse como guarda de tipo TypeScript.
- ⚠️ Las funciones de predicado deben ser funciones puras.
- ⚠️ Nombre similar pero uso diferente de `buffer` (valores individuales frente a matrices).
