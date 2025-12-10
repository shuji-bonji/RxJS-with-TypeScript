---
description: combineLatestAll toma un Higher-order Observable (Observable de Observables) y combina el último valor de cada uno cuando todos los Observables internos han emitido al menos una vez.
titleTemplate: ':title'
---

# combineLatestAll - Combinar Últimos Valores de Todos los Observables Internos

El operador `combineLatestAll` toma un **Higher-order Observable** (Observable de Observables),
**una vez que todos los Observables internos han emitido al menos una vez**, combina sus **últimos valores** y los emite como un arreglo.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// Higher-order Observable con tres Observables internos
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Combinar últimos valores una vez que todos los Observables internos han emitido al menos una vez
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Salida:
// [1, 3, 0] ← Cuando todos han emitido al menos una vez (después de 2 segundos)
// [2, 3, 0] ← 1er Observable emite 2 (después de 3 segundos)
// [2, 3, 1] ← 3er Observable emite 1 (después de 4 segundos)
```

- Recopila Observables internos cuando Higher-order Observable **completa**
- **Una vez que todos los Observables internos han emitido al menos una vez**, comienza a combinar
- Cada vez que cualquier Observable interno emite un valor, **combina todos los últimos valores** y emite

[🌐 Documentación Oficial de RxJS - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Patrones de Uso Típicos

- **Combinar últimos resultados de múltiples llamadas API**
- **Sincronizar últimos valores de múltiples entradas de formulario**
- **Integrar múltiples fuentes de datos en tiempo real**

## 🔄 Creation Function Relacionada

Mientras que `combineLatestAll` se utiliza principalmente para aplanar Higher-order Observables,
use la **Creation Function** `combineLatest` para combinaciones normales de múltiples Observables.

```ts
import { combineLatest, interval } from 'rxjs';

// Versión Creation Function (uso más común)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Consulte [Capítulo 3: Creation Functions - combineLatest](/es/guide/creation-functions/combination/combineLatest).

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| [mergeAll](/es/guide/operators/combination/mergeAll) | Suscribirse a todos los Observables internos en paralelo |
| [concatAll](/es/guide/operators/combination/concatAll) | Suscribirse a Observables internos en orden |
| [switchAll](/es/guide/operators/combination/switchAll) | Cambiar a nuevo Observable interno |
| [zipAll](/es/guide/operators/combination/zipAll) | Emparejar valores en orden correspondiente de cada Observable interno |

## ⚠️ Notas Importantes

### Higher-order Observable Debe Completar

`combineLatestAll` espera para recopilar Observables internos hasta que el Higher-order Observable (Observable externo) **completa**.

#### ❌ No se emite nada porque Higher-order Observable no completa
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // No se emite nada
```

#### ✅ Completar con take
```ts
interval(1000).pipe(
  take(3), // Completar después de 3
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Todos los Observables Internos Deben Emitir al Menos Una Vez

No se emitirán valores hasta que todos los Observables internos hayan **emitido al menos una vez**.

```ts
import { of, NEVER } from 'rxjs';
import { combineLatestAll } from 'rxjs';

// No se emite nada si incluso un Observable interno nunca emite
of(
  of(1, 2, 3),
  NEVER // Nunca emite
).pipe(
  combineLatestAll()
).subscribe(console.log); // No se emite nada
```

### Uso de Memoria

Tenga en cuenta el uso de memoria si hay muchos Observables internos, ya que **los últimos valores de todos los Observables internos se mantienen en memoria**.
