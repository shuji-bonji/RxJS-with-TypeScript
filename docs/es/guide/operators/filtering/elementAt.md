---
description: El operador elementAt es un operador de filtrado de RxJS que recupera solo el valor en una posición de índice especificada. Se comporta de manera similar al acceso a índices de arrays.
---

# elementAt - Obtener Valor en Índice Especificado

El operador `elementAt` recupera **solo el valor en la posición de índice especificada** de un Observable e inmediatamente completa el flujo. Se comporta de manera similar a `array[index]`.

## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida: 30 (valor en índice 2)
```

**Flujo de operación**:
1. 10 (índice 0) → Omitir
2. 20 (índice 1) → Omitir
3. 30 (índice 2) → Emitir y completar
4. 40, 50 no se evalúan

[🌐 Documentación Oficial de RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Patrones de Uso Típicos

- **Paginación**: Obtener primer elemento de una página específica
- **Recuperación de datos ordenados**: Obtener N-ésimo evento o mensaje
- **Pruebas y depuración**: Verificar valor en posición específica
- **Acceso tipo array**: Tratar Observable como un array

## 📚 Operadores Relacionados

- **[take](/es/guide/operators/filtering/take)** - Obtener primeros N valores
- **[first](/es/guide/operators/filtering/first)** - Obtener primer valor
- **[last](/es/guide/operators/filtering/last)** - Obtener último valor
- **[skip](/es/guide/operators/filtering/skip)** - Omitir primeros N valores
- **[takeLast](/es/guide/operators/filtering/takeLast)** - Obtener últimos N valores

## Resumen

El operador `elementAt` recupera solo el valor en la posición de índice especificada.

- ✅ Mismo comportamiento que el acceso a índices de arrays
- ✅ Ideal para obtener el N-ésimo valor
- ✅ Puede evitar errores especificando valor predeterminado
- ⚠️ Error si el índice está fuera de rango (sin valor predeterminado)
- ⚠️ Índice negativo no disponible
- ⚠️ Espera hasta alcanzar la posición para flujos asíncronos
