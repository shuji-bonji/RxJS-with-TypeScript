---
description: "El operador skipLast omite los últimos N valores en flujos Observable y emite solo valores anteriores: Perfecto para excluir datos pendientes no confirmados"
---

# skipLast - Omitir los Últimos N Valores

El operador `skipLast` **omite los últimos N valores** emitidos del Observable fuente y emite solo los valores anteriores a ellos. Mantiene los últimos N valores en un búfer hasta que el flujo se complete y emite el resto.

## 🔰 Sintaxis Básica y Uso

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omiten)
```

**Flujo de operación**:
1. El flujo emite 0, 1, 2, ...
2. Mantiene los últimos 3 valores (7, 8, 9) en búfer
3. Emite valores que exceden el tamaño del búfer (0~6)
4. Cuando el flujo se completa, los valores del búfer (7, 8, 9) se descartan sin emitir

[🌐 Documentación Oficial de RxJS - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Patrones de Uso Típicos

- **Excluir últimos datos**: Excluir últimos datos no confirmados
- **Procesamiento por lotes**: Excluir datos pendientes antes de que complete el procesamiento
- **Validación de datos**: Cuando se requiere validación en valores subsiguientes
- **Procesamiento de datos finalizados retrasados**: Cuando los últimos N elementos no están finalizados

## 📚 Operadores Relacionados

- **[skip](/es/guide/operators/filtering/skip)** - Omitir primeros N valores
- **[takeLast](/es/guide/operators/filtering/takeLast)** - Tomar solo últimos N valores
- **[take](/es/guide/operators/filtering/take)** - Tomar solo primeros N valores
- **[skipUntil](/es/guide/operators/filtering/skipUntil)** - Omitir hasta que otro Observable emita
- **[skipWhile](/es/guide/operators/filtering/skipWhile)** - Omitir mientras se cumpla la condición

## Resumen

El operador `skipLast` omite los últimos N valores en el flujo.

- ✅ Ideal cuando no se necesitan los últimos N datos
- ✅ Útil para excluir datos no confirmados
- ✅ El tamaño del búfer es solo N (eficiente en memoria)
- ✅ Requiere completación del flujo
- ⚠️ No se puede usar con flujos infinitos
- ⚠️ No hay emisión hasta que el búfer se llene con N valores
- ⚠️ A menudo necesita combinarse con `take` para hacer flujo finito
