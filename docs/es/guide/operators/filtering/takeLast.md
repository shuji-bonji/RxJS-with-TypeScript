---
description: takeLast es un operador de filtrado de RxJS que emite solo los últimos N valores cuando el flujo Observable se completa. Es ideal para escenarios donde solo se necesitan los últimos valores de todo el flujo, como obtener las últimas entradas de registro, mostrar los N elementos principales en una tabla de clasificación y resúmenes de datos finales al completarse. No se puede usar con flujos infinitos porque mantiene valores en un búfer hasta la completación.
---

# takeLast - Obtener los Últimos N Valores

El operador `takeLast` emite solo los últimos N valores cuando el flujo **se completa**. Mantiene valores en un búfer hasta que el flujo se completa, luego los emite todos a la vez.


## 🔰 Sintaxis Básica y Uso

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9
```

**Flujo de operación**:
1. El flujo emite 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. Internamente mantiene los últimos 3 valores en búfer
3. El flujo se completa
4. Emite valores de búfer 7, 8, 9 en orden

[🌐 Documentación Oficial de RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Contraste con take

`take` y `takeLast` tienen comportamientos contrastantes.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

// take: Obtener primeros N valores
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Salida: 0, 1, 2 (emite inmediatamente)

// takeLast: Obtener últimos N valores
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9 (emite después de esperar la completación)
```

| Operador | Posición de Obtención | Momento de Emisión | Comportamiento Antes de Completación |
|---|---|---|---|
| `take(n)` | Primeros n valores | Emite inmediatamente | Auto-completa después de n valores |
| `takeLast(n)` | Últimos n valores | Emite todos juntos después de completación | Mantener en búfer |


## ⚠️ Notas Importantes

> [!WARNING]
> `takeLast` **espera hasta que el flujo se complete**, por lo que no funciona con flujos infinitos. Además, si n en `takeLast(n)` es grande, consume mucha memoria.

## 🚀 Próximos Pasos

- **[take](/es/guide/operators/filtering/take)** - Aprender cómo obtener los primeros N valores
- **[last](/es/guide/operators/filtering/last)** - Aprender cómo obtener el último valor
- **[skip](/es/guide/operators/filtering/skip)** - Aprender cómo omitir los primeros N valores
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
