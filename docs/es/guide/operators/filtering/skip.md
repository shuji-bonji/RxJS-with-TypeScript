---
description: "El operador skip omite el primer número especificado de valores del flujo Observable y sólo emite los valores siguientes. Esto resulta útil cuando se desea ignorar los datos iniciales o saltarse el periodo de calentamiento."
---

# skip - skip los primeros N valores

El operador `skip` omite el **primer número especificado** de valores del flujo y sólo muestra los valores siguientes.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Salida.: 3, 4, 5, 6, 7, ...
```

- Omitir los tres primeros casos (0, 1, 2).
- La cuarta y siguientes entradas (3, 4, 5, ...) son todas de salida
- El flujo se completa en el tiempo de finalización original

[Documentación oficial de RxJS - `skip`](https://rxjs.dev/api/operators/skip)

## 🆚 Contraste con take

`skip` y `take` tienen comportamientos opuestos.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0desde9De a

// take: PrimeraNObtener el primero
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Salida.: 0, 1, 2

// skip: PrimeraNSaltar la primera pieza
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Salida.: 3, 4, 5, 6, 7, 8, 9

// Combinación: Primera3Omitir la primera pieza y supervisar sólo la siguiente3Obtener el primero
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Salida.: 3, 4, 5
```

| Operador | Operación | Tiempo de finalización |
|---|---|---|
| take(n) | Se toman las primeras n piezas | Finalización automática después de tomar n piezas. |
| skip(n)` | Saltar los primeros n | Al finalizar el flujo original |

## 💡 Patrón típico de utilización

1. **Omitir valores iniciales**.

```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Omitir valores iniciales, supervisar sólo los cambios
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`Estado cambiado: ${value}`);
   });

   state$.next(1); // Salida.: Estado cambiado: 1
   state$.next(2); // Salida.: Estado cambiado: 2
   ```

2. **Omitir el periodo de calentamiento**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simulación de los datos del sensor
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Primera10El caso (1segundos) omitido como periodo de calibración
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
     { id: 1, name: 'Item 1' },
     { id: 2, name: 'Item 2' },
     { id: 3, name: 'Item 3' },
     { id: 4, name: 'Item 4' },
     { id: 5, name: 'Item 5' },
     { id: 6, name: 'Item 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // 0-indexed

   // Página2Obtener los elementos de (elementos5y6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Salida.: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```

## 🧠 Ejemplo práctico de código (contador)

Primera3Omitir un clic,4Este es un ejemplo para contar sólo el segundo clic y los siguientes.

```

ts.
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// Creación de elementos de interfaz de usuario
const container = document.createElement('div');
document.body.appendChild(contenedor);

const button = document.createElement('button');
button.textContent = 'clic';
container.appendChild(button);

const count = document.createElement('div');
count.style.marginTop = '10px';
counter.textContent = 'count: 0';
container.appendChild(count);

const mensaje = document.createElement('div');
message.style.marginTop = '5px';
message.style.colour = 'gris';
message.textContent = 'Se omitirán los tres primeros clics';
container.appendChild(mensaje);

// Evento click
fromEvent(botón, 'clic').pipe(
  skip(3), // skip los 3 primeros clics
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Conteo: ${count}`;
  if (count === 1) {
    message.textContent = `¡El recuento comienza tras el 4º clic! ;
    message.style.colour = 'verde';
  }
});

```

Este código ignora los primeros3El código ignora los dos primeros clics,4El código ignora el primer clic y empieza a contar a partir del segundo clic como "1El código ignora el primer clic y empieza a contar a partir del segundo clic como "1".

## 🎯 skip y skipWhile Diferencia entre

```

ts.
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const números$ = of(1, 2, 3, 4, 5, 6);

// skip: primera N por número
números$.pipe(
  skip(3)
).subscribe(console.log);
// salida: 4, 5, 6

// skipWhile: saltar mientras se cumplan las condiciones
números$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Salida: 4, 5, 6

```

| Operador | Saltar condiciones | Caso de uso |
|---|---|---|
| `skip(n)` | PrimeranSaltar pieza por número | Omitir un número fijo |
| `skipWhile(predicate)` | Omitir mientras se cumplen las condiciones | Salto basado en condiciones |
| `skipUntil(notifier$)` | Saltar hasta que otroObservableSaltar hasta que se dispare otro | Salto basado en el tiempo |

## 📋 Uso seguro

TypeScript Este es un ejemplo de una implementación de tipo seguro que hace uso de los genéricos en

```

ts.
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs';

interfaz Usuario {
  id: número;
  nombre: cadena;
  rol: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable,.
  page: número,.
  pageSize: número
): Observable {
  return users$.pipe(
    skip(página * tamañoPágina),.
    take(tamañoPágina)
  );
}

// Ejemplo de uso
const usuarios$ = from([.
  { id: 1, nombre: 'Alice', rol: 'admin' as const }
  { id: 2, nombre: 'Bob', rol: 'usuario' as const }
  { id: 3, name: 'Charlie', role: 'usuario' as const }
  { id: 4, name: 'Dave', role: 'admin' as const }
  { id: 5, name: 'Eve', role: 'usuario' as const }
] as Usuario[]);.

// Obtener página 1 (segunda página, indexada 0)
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Salida: Charlie (usuario), Dave (admin)

```

## ⚠️ Un error común

> [!NOTE]
> `skip` es saltar hasta el primerNy no completa el flujo. Para flujos infinitos, use `take` la condición de salida debe combinarse con

### error.: En un flujo infinito skip Utilice sólo

```

ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Mal ejemplo: el flujo infinito continúa tal cual
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... ... continúa eternamente.

```

### Positivo: take Establezca la condición de fin en combinación con

```

ts.
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Buen ejemplo: limitar el número de tomas tras skip
interval(1000).pipe(
  skip(5), take(3)
  take(3)
).subscribe({
  next: console.log,.
  complete: () => console.log('completado')
});
// 5, 6, 7, completo.
```

## 🎓 Resumen

### Cuándo se debe usar skip.
- ✅ Si se desea ignorar el valor inicial o los primeros N datos.
- ✅ Cuando se desea omitir el valor inicial de un BehaviorSubject
- ✅ Si desea obtener los datos de una página específica en la paginación
- ✅ Si desea omitir el periodo de calibración del sensor

### Cuando se combina con take
- ✅ Si desea obtener sólo un rango específico de datos
- ✅ Si desea adquirir datos en la parte central de un flujo infinito

### Notas.
- ⚠️ En flujos infinitos, usar en combinación con `take` para establecer la condición de fin.
- ⚠️ `skip(0)` funciona igual que el flujo original (no se salta nada)
- ⚠️ Si el número de saltos es mayor que el número total de datos, finaliza sin emitir nada.

## 🚀 Próximos pasos.

- **[take](. /take)** - aprende a obtener los N primeros valores.
- **[first](. /first)** - aprende a obtener el primer valor o el primer valor que satisface la condición.
- last](. /last)** - aprende a obtener el último valor.
- filter](. /filter)** - aprende a filtrar en base a condiciones
- **[filtro-operador-casos-prácticos](. /practical-use-cases)** - aprende a utilizar casos de uso reales
