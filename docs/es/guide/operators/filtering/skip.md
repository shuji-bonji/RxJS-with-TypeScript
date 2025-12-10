---
description: El operador skip omite el número especificado de primeros valores del flujo Observable y emite solo los valores subsiguientes. Esto es útil cuando deseas ignorar datos iniciales u omitir un período de calentamiento.
---

# skip - Omitir los Primeros N Valores

El operador `skip` omite el **número especificado de primeros** valores del flujo y emite solo los valores subsiguientes.


## 🔰 Sintaxis Básica y Uso

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Salida: 3, 4, 5, 6, 7, ...
```

- Omite los primeros 3 valores (0, 1, 2)
- Los valores cuarto y subsiguientes (3, 4, 5, ...) se emiten todos
- El flujo se completa en el momento de completación original

[🌐 Documentación Oficial de RxJS - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Contraste con take

`skip` y `take` tienen comportamientos contrastantes.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

// take: Obtener los primeros N valores
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Salida: 0, 1, 2

// skip: Omitir los primeros N valores
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Salida: 3, 4, 5, 6, 7, 8, 9

// Combinación: Omitir los primeros 3 y obtener los siguientes 3
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Salida: 3, 4, 5
```

| Operador | Comportamiento | Momento de Completación |
|---|---|---|
| `take(n)` | Obtener los primeros n valores | Se completa automáticamente después de n valores |
| `skip(n)` | Omitir los primeros n valores | Cuando el flujo original se completa |


## 💡 Patrones de Uso Típicos

1. **Omitir Valores Iniciales**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Omitir valor inicial y monitorear solo cambios
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`Estado cambió: ${value}`);
   });

   state$.next(1); // Salida: Estado cambió: 1
   state$.next(2); // Salida: Estado cambió: 2
   ```

2. **Omitir Período de Calentamiento**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simular datos de sensor
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Omitir los primeros 10 valores (1 segundo) como período de calibración
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Valor del sensor: ${data.toFixed(2)}`);
   });
   ```

3. **Paginación**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Elemento 1' },
     { id: 2, name: 'Elemento 2' },
     { id: 3, name: 'Elemento 3' },
     { id: 4, name: 'Elemento 4' },
     { id: 5, name: 'Elemento 5' },
     { id: 6, name: 'Elemento 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // 0-indexado

   // Obtener los elementos en la página 2 (elementos 5 y 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Salida: { id: 5, name: 'Elemento 5' }, { id: 6, name: 'Elemento 6' }
   ```


## ⚠️ Errores Comunes

> [!NOTE]
> `skip` solo omite los primeros N valores y no completa el flujo. En flujos infinitos, combinar con `take` para establecer la condición de terminación.

## 📚 Operadores Relacionados

- **[take](/es/guide/operators/filtering/take)** - Obtener los primeros N valores
- **[first](/es/guide/operators/filtering/first)** - Obtener el primer valor o el primer valor que satisface una condición
- **[last](/es/guide/operators/filtering/last)** - Obtener el último valor
- **[filter](/es/guide/operators/filtering/filter)** - Filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
