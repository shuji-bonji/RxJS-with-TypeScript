---
description: find es un operador de filtrado de RxJS que encuentra y emite el primer valor que satisface una condición e inmediatamente completa el flujo. Es ideal para escenarios donde deseas buscar un elemento específico de un array o lista, como búsqueda de usuarios, verificación de inventario y detección de registros de errores. Si no se encuentra ningún valor, emite undefined, y en TypeScript el valor de retorno es de tipo T | undefined.
titleTemplate: ':title | RxJS'
---

# find - Encontrar el Primer Valor que Satisface una Condición

El operador `find` encuentra y emite el **primer valor que satisface una condición** e inmediatamente completa el flujo. Si no se encuentra ningún valor, emite `undefined`.


## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Salida: 8 (primer número par)
```

**Flujo de operación**:
1. Verificar 1, 3, 5, 7 → No satisfacen la condición
2. Verificar 8 → Satisface la condición → Emitir 8 y completar
3. 9, 10 no se evalúan

[🌐 Documentación Oficial de RxJS - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Contraste con first

`find` y `first` son similares pero se usan de manera diferente.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Primer valor que satisface la condición (la condición es opcional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Salida: 7

// find: Primer valor que satisface la condición (la condición es requerida)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Salida: 7
```

| Operador | Especificación de Condición | Cuando no se Encuentra Valor | Caso de Uso |
|---|---|---|---|
| `first()` | Opcional | Error (`EmptyError`) | Obtener primer valor |
| `first(predicate)` | Opcional | Error (`EmptyError`) | Obtención condicional |
| `find(predicate)` | Requerido | Emite `undefined` | Búsqueda/verificación de existencia |


## 🚀 Próximos Pasos

- **[first](/es/guide/operators/filtering/first)** - Aprender cómo obtener el primer valor
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - Aprender cómo obtener el índice del primer valor que satisface la condición (documentación oficial)
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
