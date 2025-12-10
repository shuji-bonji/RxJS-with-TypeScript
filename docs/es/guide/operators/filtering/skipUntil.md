---
description: El operador skipUntil omite todos los valores del Observable original hasta que otro Observable emita un valor, luego emite valores normalmente. Es útil para inicio retrasado basado en tiempo o procesamiento después de que ocurra un evento específico.
---

# skipUntil - Omitir Hasta que Otro Observable Emita

El operador `skipUntil` **omite todos los valores del Observable fuente** hasta que un Observable especificado (disparador de notificación) emita su primer valor. Después de que el disparador de notificación emita, los valores subsiguientes se emiten normalmente.


## 🔰 Sintaxis Básica y Uso

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Emitir valor cada 0.5 segundos
const notifier$ = timer(2000); // Emitir valor después de 2 segundos

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, 8, ...
// (Los valores de los primeros 2 segundos 0, 1, 2, 3 se omiten)
```

**Flujo de operación**:
1. `source$` emite 0, 1, 2, 3 → todos omitidos
2. Después de 2 segundos, `notifier$` emite un valor
3. Los valores subsiguientes de `source$` (4, 5, 6, ...) se emiten normalmente

[🌐 Documentación Oficial de RxJS - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Contraste con takeUntil

`skipUntil` y `takeUntil` tienen comportamientos contrastantes.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Emitir valor cada 0.5 segundos
const notifier$ = timer(2000); // Emitir valor después de 2 segundos

// takeUntil: Tomar valores hasta notificación
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Salida: 0, 1, 2, 3 (se detiene después de 2 segundos)

// skipUntil: Omitir valores hasta notificación
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, ... (comienza después de 2 segundos)
```

| Operador | Comportamiento | Momento de Completación |
|---|---|---|
| `takeUntil(notifier$)` | **Tomar** valores hasta notificación | Auto-completa cuando se notifica |
| `skipUntil(notifier$)` | **Omitir** valores hasta notificación | Cuando el flujo fuente se completa |


## 🚀 Próximos Pasos

- **[skip](/es/guide/operators/filtering/skip)** - Aprender cómo omitir los primeros N valores
- **[take](/es/guide/operators/filtering/take)** - Aprender cómo tomar los primeros N valores
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
