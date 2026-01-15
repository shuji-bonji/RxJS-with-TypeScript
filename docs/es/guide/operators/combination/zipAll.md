---
description: zipAll es un operador que toma un Higher-order Observable (Observable de Observables) y empareja los valores ordenados correspondientes de cada Observable interno y los emite como un arreglo.
titleTemplate: ':title'
---

# zipAll - Emparejar por Orden

El operador `zipAll` toma un **Higher-order Observable** (Observable de Observables),
**empareja los valores ordenados correspondientes de cada Observable interno** y los emite como un arreglo.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable con tres Observables internos
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Emparejar valores ordenados correspondientes de cada Observable interno
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Salida:
// [0, 0, 0] ← Todos los 1ros valores
// [1, 1, 1] ← Todos los 2dos valores
// (Completa aquí: 3er Observable solo emite 2 valores)
```

- Recopila Observables internos cuando Higher-order Observable **completa**
- **Empareja los valores del mismo índice** de cada Observable interno
- **Cuando el Observable interno más corto completa**, todo completa

[🌐 Documentación Oficial de RxJS - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 Patrones de Uso Típicos

- **Emparejar múltiples respuestas API en secuencia**
- **Comparar valores del mismo tiempo de múltiples streams**
- **Combinar resultados de procesamiento paralelo en secuencia**

## 🧠 Ejemplo de Código Práctico

Ejemplo de emparejar los valores correspondientes de múltiples contadores

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Crear tres contadores con diferentes velocidades
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Lento: ${n}`)),
  interval(500).pipe(take(5), map(n => `Normal: ${n}`)),
  interval(300).pipe(take(6), map(n => `Rápido: ${n}`))
);

// Emparejar valores ordenados correspondientes de cada contador
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Salida:
// [Lento: 0, Normal: 0, Rápido: 0]
// [Lento: 1, Normal: 1, Rápido: 1]
// [Lento: 2, Normal: 2, Rápido: 2]
// [Lento: 3, Normal: 3, Rápido: 3]
// (Completa aquí: contador "Lento" solo emite 4 valores)
```

## 🔄 Creation Function Relacionada

Mientras que `zipAll` se utiliza principalmente para aplanar Higher-order Observables,
use la **Creation Function** `zip` para emparejar normalmente múltiples Observables.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Versión Creation Function (uso más común)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

Consulte [Capítulo 3: Creation Functions - zip](/es/guide/creation-functions/combination/zip).

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| [combineLatestAll](/es/guide/operators/combination/combineLatestAll) | Combinar últimos valores de todos los Observables internos |
| [mergeAll](/es/guide/operators/combination/mergeAll) | Suscribirse a todos los Observables internos en paralelo |
| [concatAll](/es/guide/operators/combination/concatAll) | Suscribirse a Observables internos en orden |
| [switchAll](/es/guide/operators/combination/switchAll) | Cambiar a nuevo Observable interno |

## 🔄 zipAll vs combineLatestAll

| Operador | Método de Combinación | Tiempo de Completación |
|---|---|---|
| `zipAll` | Empareja valores en el **mismo índice** | Cuando el Observable interno **más corto** completa |
| `combineLatestAll` | Combina **últimos valores** | Cuando **todos** los Observables internos completan |

```ts
// zipAll: [0mo, 0mo, 0mo], [1ro, 1ro, 1ro], ...
// combineLatestAll: [último, último, último], [último, último, último], ...
```

## ⚠️ Notas Importantes

### Higher-order Observable Debe Completar

`zipAll` espera para recopilar Observables internos hasta que el Higher-order Observable (Observable externo) **completa**.

#### ❌ No se emite nada porque Higher-order Observable no completa
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // No se emite nada
```

#### ✅ Completar con take
```ts
interval(1000).pipe(
  take(3), // Completar después de 3
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Completa con Observable Interno Más Corto

Cuando el **Observable interno más corto completa**, todo completa.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 valores
  of(1, 2)           // 2 valores ← Más corto
).pipe(
  zipAll()
).subscribe(console.log);

// Salida: [1, 1], [2, 2]
// (Completa en 2. 3, 4, 5 no se usan)
```

### Backpressure (Uso de Memoria)

Cuando los Observables internos emiten a diferentes velocidades, **los valores de Observables internos más rápidos se acumulan en memoria**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// Los valores del contador rápido (100ms) se acumulan en memoria mientras esperan al contador lento (10000ms)
of(
  interval(10000).pipe(take(3)), // Lento
  interval(100).pipe(take(100))  // Rápido
).pipe(
  zipAll()
).subscribe(console.log);
```

Si la diferencia de velocidad es grande, preste atención al uso de memoria.
