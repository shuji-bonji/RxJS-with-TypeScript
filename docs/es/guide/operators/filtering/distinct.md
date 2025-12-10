---
description: El operador distinct elimina todos los valores duplicados y emite solo valores únicos que nunca han sido emitidos. Se debe tener cuidado con flujos infinitos, ya que internamente usa Set para almacenar valores previamente emitidos.
---

# distinct - Eliminar Todos los Valores Duplicados

El operador `distinct` monitorea todos los valores emitidos por Observable y emite **solo valores que nunca han sido emitidos antes**. Internamente, usa Set para recordar valores previamente emitidos.


## 🔰 Sintaxis Básica y Uso

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5
```

- Elimina duplicados en todo el flujo
- Una vez que un valor se emite, se ignora sin importar cuántas veces aparezca posteriormente
- `distinctUntilChanged` elimina solo duplicados **consecutivos**, mientras que `distinct` elimina **todos** los duplicados

[🌐 Documentación Oficial de RxJS - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Diferencia con distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Eliminar solo duplicados consecutivos
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Salida: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Eliminar todos los duplicados
values$.pipe(
  distinct()
).subscribe(console.log);
// Salida: 1, 2, 3
```

| Operador | Objetivo de Eliminación | Caso de Uso |
|---|---|---|
| `distinctUntilChanged` | Solo duplicados consecutivos | Campos de entrada, datos de sensores |
| `distinct` | Todos los duplicados | Lista de valores únicos, lista de ID |


## 🚀 Próximos Pasos

- **[distinctUntilChanged](/es/guide/operators/filtering/distinctUntilChanged)** - Aprender cómo eliminar solo duplicados consecutivos
- **[distinctUntilKeyChanged](/es/guide/operators/filtering/distinctUntilKeyChanged)** - Aprender cómo comparar objetos por clave
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
