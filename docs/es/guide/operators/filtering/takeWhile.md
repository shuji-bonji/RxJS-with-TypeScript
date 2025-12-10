---
description: "takeWhile es un operador de filtrado de RxJS que continúa tomando valores mientras se cumpla la condición especificada y completa el flujo cuando la condición se vuelve falsa. Es ideal para situaciones donde deseas controlar un flujo con condiciones dinámicas, como adquisición de datos hasta un umbral, procesamiento basado en prioridad, paginación, etc. La opción inclusive permite incluir valores para los cuales la condición se vuelve falsa."
---

# takeWhile - Tomar Valores Mientras se Cumpla la Condición

El operador `takeWhile` continúa tomando valores **mientras se cumpla la condición especificada**, y completa el flujo cuando la condición se vuelve `false`.


## 🔰 Sintaxis Básica y Uso

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Completo')
});
// Salida: 0, 1, 2, 3, 4, Completo
```

**Flujo de operación**:
1. Se emite 0 → `0 < 5` es `true` → Emite
2. Se emite 1 → `1 < 5` es `true` → Emite
3. Se emite 2 → `2 < 5` es `true` → Emite
4. Se emite 3 → `3 < 5` es `true` → Emite
5. Se emite 4 → `4 < 5` es `true` → Emite
6. Se emite 5 → `5 < 5` es `false` → Completa (5 no se emite)

[🌐 Documentación Oficial de RxJS - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Contraste con take

`take` y `takeWhile` tienen diferentes condiciones de adquisición.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Control por conteo
source$.pipe(
  take(5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4

// takeWhile: Control por condición
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4
```

| Operador | Método de Control | Condición de Completación | Último Valor |
|---|---|---|---|
| `take(n)` | Conteo | Después de n valores | Incluye el valor n-ésimo |
| `takeWhile(predicate)` | Función de condición | Cuando la condición se vuelve `false` | No incluye valor que se volvió `false`* |

\* Por defecto, el valor que se volvió `false` no se emite, pero se puede incluir con la opción `inclusive: true`


## 🚀 Próximos Pasos

- **[take](/es/guide/operators/filtering/take)** - Aprender cómo tomar los primeros N valores
- **[takeLast](/es/guide/operators/filtering/takeLast)** - Aprender cómo tomar los últimos N valores
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
