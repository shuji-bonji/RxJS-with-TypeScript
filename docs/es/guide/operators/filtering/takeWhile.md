---
description: "takeWhile es un operador de filtrado de RxJS que continúa recuperando valores mientras se cumple la condición especificada y completa el flujo cuando la condición se convierte en falsa. Es ideal para situaciones en las que se desea controlar el flujo con condiciones dinámicas, como la obtención de datos hasta un umbral, el procesamiento basado en la prioridad, la paginación, etc. La opción inclusiva permite incluir valores para los que la condición es falsa."
---

# takeWhile - obtener el valor mientras se cumple la condición

El operador `takeWhile` continúa tomando valores **mientras** se cumple la condición especificada y completa el flujo cuando la condición es `false`.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Finalización')
});
// Salida: 0, 1, 2, 3, 4, Finalización
```

**Flujo de operación**:.
1. 0 emitido → `0 < 5` es `true` → salida
2. 1 emitido → `1 < 5` es `true` → salida
3. 2 salidas → `2 < 5` es `true` → salida
4. 3 emisiones → `3 < 5` es `true` → salida
5. 4 problemas → `4 < 5` es `verdadero` → Salida
6. 5 incidencias → `5 < 5` es `false` → completo (5 no tiene salida)

[🌐 Documentación oficial de RxJS - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)

## 🆚 Contraste con take

`take` y `takeWhile` tienen diferentes condiciones de take.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Controlado por número de piezas
source$.pipe(
  take(5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4

// takeWhile: Controlado por condiciones
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4
```

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Finalización')
});
// Salida: 0, 1, 2, 3, 4, Finalización
```

\* Los valores que por defecto son `false` no se muestran, pero pueden incluirse con la opción `inclusive: true

## 🎯 opción inclusiva.

Especifique `inclusive: true` si desea incluir valores para los que la condición es `false`.


```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Por defecto (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5(condiciones)false(también incluido)5(también incluido)
```

## 💡 Patrón de utilización típico.

1. **Adquisición de datos hasta un valor umbral**.

```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Simulación de sensores de temperatura
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // 30Registrado sólo durante menos de 1,5 grados Celsius
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Temperatura: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Advertencia.: La temperatura30ha superado los 1,5 grados Celsius.！')
   });
   ```

2. **Procesamiento condicional de matrices**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Procesado sólo mientras la prioridad eshighProcesado sólo durante
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Tarea${task.id}se está procesando`);
   });
   // Salida: Tarea1se está procesando, Tarea2se está procesando
   ```

3. **Proceso de paginación**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // hasMorese está procesandotrueCarga de páginas sólo durante
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Página${page.pageNumber}está cargada`);
   });
   // Salida: Página1~.5está cargada
   ```

## 🧠 Ejemplo práctico de código (limitación de la cuenta ascendente)

Este es un ejemplo de cómo continuar contando hasta que se alcanza una condición específica.

```

ts.
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// Creación de elementos UI
const container = document.createElement('div');.
document.body.appendChild(contenedor);

const startButton = document.createElement('button');
startButton.textContent = 'Empieza a contar';
container.appendChild(startButton);

const count = document.createElement('div');
count.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'count: 0';
container.appendChild(count);

const mensaje = document.createElement('div');
message.style.marginTop = '5px';
message.style.colour = 'gris';
message.textContent = 'Sigue contando por menos de 10';
container.appendChild(mensaje);

// Empezar a contar al pulsar el botón
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),.
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Cuenta: ${cuenta}`;
    startButton.disabled = true;
  },.
  complete: () => {
    message.textContent = `¡Has llegado a 10, has completado! ;
    message.style.colour = 'verde';
    startButton.disabled = false;
  }
});

```

Este código es0a9y se completa automáticamente justo antes de llegar a10y se completa automáticamente justo antes de alcanzar

## 🎯 skipWhile En contraste con

`takeWhile` y `skipWhile` tiene un comportamiento opuesto.

```

ts.
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const números$ = range(0, 10);

// takeWhile: tomar mientras se cumple la condición
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4

// skipWhile: saltar mientras se cumplan las condiciones
números$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9

```

| Operador | Operación | Tiempo de realización |
|---|---|---|
| `takeWhile(predicate)` | Mientras se cumplen las condiciones**Adquisición** | Mientras se cumplen las condiciones`false`Cuando |
| `skipWhile(predicate)` | Mientras se cumplen las condiciones**Saltar** | Al finalizar el flujo original |

## 📋 Uso seguro

TypeScript Este es un ejemplo de una implementación de tipo seguro que utiliza genéricos en

```

ts.
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interfaz SensorReading {
  timestamp: Fecha;
  valor: número;
  unidad: cadena;.
  status: 'normal' | 'warning' | 'critical'; }
}

function getReadingsUntilWarning(
  lecturas$: Observable
): Observable\<SensorLectura> {
  return lecturas$.pipe(
    takeWhile(lectura => lectura.estado === 'normal')
  );
}

// Ejemplo de uso
const lecturas$ = from([.
  { timestamp: new Date(), valor: 25, unidad: '°C', estado: 'normal' as const }
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const }
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const }
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const }
] as SensorReading[]);.

getReadingsUntilWarning(lecturas$).subscribe(lecturas => {
  console.log(`${valor.lectura}${unidad.lectura} - ${estado.lectura}`);
});
// Salida:.
// 25°C - normal
// 28°C - normal

```

## 🔄 takeWhile y filter La diferencia entre

`takeWhile` es que la finalización es `filter` difiere de la de

```

ts.
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const números$ = range(0, 10);

// filter: sólo se pasan los valores que cumplen la condición (el flujo continúa)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filtro complete')
});
// Salida: 0, 1, 2, 3, 4, filter completo

// takeWhile: sólo mientras se cumpla la condición (condición false para completar)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('takeWhile completado')
});
// Salida: 0, 1, 2, 3, 4, takeWhile completo

```

| Operador | Operación | Finalización de flujos |
|---|---|---|
| `filter(predicate)` | Sólo se pasan los valores que cumplen las condiciones | Al completarse el flujo original |
| `takeWhile(predicate)` | Se adquieren mientras se cumple la condición | Mientras se cumplen las condiciones`false`Cuando |

## ⚠️ Un error común

> [!NOTE]
> `takeWhile` es que si la condición no se cumple desde el principio hasta `false` entonces se completa sin ninguna salida. Compruebe que la condición se establece adecuadamente.

### Error: Si la condición se establece desde el principio a false

```

ts.
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Mal ejemplo: condición false en el primer valor.
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);.
// No sale nada (condición false en el primer valor 5)

```

### Correcto: Compruebe que las condiciones

```

ts.
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Buen ejemplo: establece las condiciones correctamente
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);.
// Salida: 0, 1, 2, 3, 4
```

## 🎓 Resumen

### Cuándo debe usarse takeWhile.
- ✅ Si se desea controlar el flujo en condiciones dinámicas.
- ✅ Si quieres tomar datos hasta que se alcance un valor umbral
- ✅ Si sólo desea procesar los datos mientras dure una condición específica
- ✅ Si necesita una finalización anticipada basada en una condición

### Cuándo se debe utilizar take.
- ✅ Cuando el número de piezas a adquirir es fijo.
- ✅ Cuando se necesita un límite de piezas simple.

### Cuándo utilizar filter.
- ✅ Si desea extraer de todo el flujo sólo los valores que coincidan con los criterios.
- ✅ Cuando no se desea completar el flujo.

### Notas.
- ⚠️ Si la condición es `false` desde el principio, se completa sin ninguna salida.
- ⚠️ Por defecto, los valores donde la condición es `false` no se emiten (incluidos con `inclusive: true`)
- ⚠️ Si la condición es siempre `true` en un flujo infinito, continúa para siempre

## 🚀 Siguiente paso.

- **[take](./take)** - aprende a obtener los N primeros valores.
- **[takeLast](./takeLast)** - aprende a obtener los últimos N valores.
- takeUntil](../utility/takeUntil)** - aprende a tomar valores hasta que se dispara otro Observable.
- filter](./filter)** - aprende a filtrar en base a condiciones
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - aprende casos de uso reales
