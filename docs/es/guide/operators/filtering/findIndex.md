---
description: El operador findIndex es un operador de filtrado de RxJS que devuelve el índice del primer valor que satisface la condición. Si no se encuentra, devuelve -1.
titleTemplate: ':title | RxJS'
---

# findIndex - Obtener Índice del Primer Valor que Coincide con la Condición

El operador `findIndex` devuelve **el índice del primer valor que satisface la condición** e inmediatamente completa el flujo. Si el valor no se encuentra, devuelve `-1`.

## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Salida: 4 (índice del primer número par 8)
```

**Flujo de operación**:
1. 1 (índice 0) → Impar, omitir
2. 3 (índice 1) → Impar, omitir
3. 5 (índice 2) → Impar, omitir
4. 7 (índice 3) → Impar, omitir
5. 8 (índice 4) → Par, emitir índice 4 y completar

[🌐 Documentación Oficial de RxJS - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Patrones de Uso Típicos

- **Localizar posición en array**: Obtener posición de elemento que coincide con condición específica
- **Verificar orden**: Determinar en qué posición aparece un elemento que coincide con una condición
- **Ordenamiento de datos**: Procesamiento usando información de índice
- **Verificación de existencia**: Verificar existencia por si es -1 o no

## 🆚 Comparación con Operadores Similares

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Devolver índice del primer valor que coincide con la condición
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Salida: 2 (índice de 30)

// find: Devolver primer valor que coincide con la condición
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Salida: 30

// elementAt: Devolver valor en índice especificado
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida: 30
```

| Operador | Argumento | Valor de Retorno | Cuando no se Encuentra |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Función de condición | Índice (número) | `-1` |
| `find(predicate)` | Función de condición | Valor en sí | `undefined` |
| `elementAt(index)` | Índice | Valor en sí | Error (sin valor predeterminado) |

## 📚 Operadores Relacionados

- **[find](/es/guide/operators/filtering/find)** - Obtener primer valor que coincide con la condición
- **[elementAt](/es/guide/operators/filtering/elementAt)** - Obtener valor en índice especificado
- **[first](/es/guide/operators/filtering/first)** - Obtener primer valor
- **[filter](/es/guide/operators/filtering/filter)** - Obtener todos los valores que coinciden con la condición

## Resumen

El operador `findIndex` devuelve el índice del primer valor que coincide con la condición.

- ✅ Comportamiento similar a `Array.findIndex()` de JavaScript
- ✅ Ideal cuando se necesita información de índice
- ✅ Devuelve `-1` si no se encuentra (no es un error)
- ✅ Se completa inmediatamente cuando se encuentra
- ⚠️ El valor de retorno es siempre tipo `number` (-1 o entero ≥ 0)
- ⚠️ Usar `find` si necesitas el valor en sí
