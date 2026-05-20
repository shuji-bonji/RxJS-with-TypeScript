---
description: "El operador distinct elimina todos los valores duplicados y sólo emite valores únicos que nunca han sido emitidos. Hay que tener cuidado con los flujos infinitos, ya que internamente utilizan Set para almacenar valores previamente existentes."
---

# distinct - elimina todos los valores duplicados

El operador `distinct` monitoriza todos los valores emitidos por el Observable y emite **sólo** valores que nunca antes han sido emitidos. Internamente Set se utiliza para recordar valores publicados anteriormente.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Salida.: 1, 2, 3, 4, 5
```

- Elimina los duplicados en todo el flujo.
- Una vez que se emite un valor, se ignora sin importar cuántas veces aparezca después
- distinctUntilChanged` sólo elimina los duplicados **consecutivos**, mientras que distinct` elimina **todos los duplicados**.

[🌐 Documentación oficial de RxJS - `distinct`](https://rxjs.dev/api/operators/distinct)

## 🆚 Diferencias con distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Eliminar sólo los duplicados consecutivos
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Salida.: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Eliminar todos los duplicados
values$.pipe(
  distinct()
).subscribe(console.log);
// Salida.: 1, 2, 3
```

| Operador. | Objetivo de eliminación | Caso de uso. |
|---|---|---|
| distinctUntilChanged`. | Sólo duplicados consecutivos | Campos de entrada, datos del sensor. |
| distinct`. | Todos los duplicados | Lista de valores únicos, lista de ID |

## 🎯 Personalización de la comparación con keySelector

Utilice la función `keySelector` para determinar duplicados en ciertas propiedades de un objeto.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (updated)' } as User, // Eliminar todos los duplicados con el mismoID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // IDDuplicados determinados por
).subscribe(console.log);
// Salida.:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```

## 💡 Patrón de utilización típico.

1. **Obtener una lista de identificadores únicos**.

```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Usuarios únicosIDSólo se recuperan los usuarios únicos
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`User ID: ${userId}`);
   });
   // Salida.: 1, 2, 3
   ```

2. **Extraer tipos de eventos únicos de los registros de eventos**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // UICrear elementos dinámicamente
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Button 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Button 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Entrada.';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Fusionar múltiples flujos de eventos y extraer tipos de eventos únicos
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // 3Finaliza cuando todos los tipos de eventos están disponibles
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Unique event: ${eventType}\n`;
       console.log(`Unique event: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Todos los tipos de eventos detectados';
     }
   });
   ```

## 🧠 Ejemplo práctico de código (entrada de etiquetas)

Eliminar automáticamente los duplicados de las etiquetas introducidas por el usuarioUIEjemplo de.

```

ts.
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Creación de elementos de interfaz de usuario
const container = document.createElement('div');
document.body.appendChild(contenedor);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Introducir etiqueta e Intro';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// tagAddTagStream
const tagSubject$ = new Subject();.

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase())),.
  distinct() // eliminar etiquetas duplicadas
).subscribe(etiqueta => {
  const li = document.createElement('li');
  li.textContent = etiqueta;
  tagList.appendChild(li);
});

// Añadir una etiqueta con la tecla Intro
fromEvent\<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (valor) {
      tagSubject$.next(valor);
      tagInput.value = '';
    }
  }
});

```

Este código garantiza que la misma etiqueta sólo se añada a la lista una vez, aunque se introduzca varias veces.

> [!WARNING] Notas sobre el código de producción
> El ejemplo anterior es para simplificar la explicación. `fromEvent` Se ha omitido la cancelación de la suscripción. En el código real, el `takeUntil(destroy$)`o`take(N)`o `Subscription.unsubscribe()` para gestionar explícitamente el ciclo de vida. Detalles: [Superar las dificultades: Gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)

## ⚠️ Nota sobre el uso de la memoria

> [!WARNING]
> `distinct` El operador puede internamente **Set** almacenar todos los valores existentes. Su uso con flujos infinitos puede provocar fugas de memoria.

### Problema.: Fuga de memoria en flujos infinitos.

```

ts.
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Mal ejemplo: uso de distinct en un flujo infinito
interval(100).pipe(
  map(n => n % 10), // ciclos 0-9
  distinct() // sólo emite los 10 primeros, luego sigue almacenándolos en memoria
).subscribe(console.log);.
// Salida: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// después no sale nada, pero se sigue guardando el Set

```

### Solución.: flushes En el parámetroSetBorrar la

```

ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Buen ejemplo: borrar periódicamente Set
interval(100).pipe(
  map(n => n % 5),.
  distinct(
    valor => valor,.
    timer(1000) // Borrar Set cada segundo.
  )
).subscribe(console.log);.
// 0, 1, 2, 3, 4 se reemiten cada segundo

```

### Práctica recomendada

1. **Uso en flujos finitos**: HTTP Respuestas, conversión desde matrices, etc.
2. **flushes Utilizar para**: Borrar periódicamente para flujos infinitos
3. **distinctUntilChanged Considerar**: Utilizar si sólo se van a eliminar duplicados contiguos

## 📋 Uso seguro

TypeScript Este es un ejemplo de una implementación de tipo seguro que hace uso de los genéricos en

```

ts.
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interfaz Producto {
  id: número;
  nombre: string
  categoryId: número;
}

function getUniqueCategories(
  productos$: Observable\<Producto>
): Observable {
  return productos$.pipe(
    distinct(producto => producto.categoryId)
  ).pipe(
    map(producto => producto.categoríaId)
  );
}

// Ejemplo de uso
import { of } from 'rxjs';.

const productos$ = of(
  { id: 1, name: 'Portátil', categoryId: 10 } as Producto,.
  { id: 2, name: 'Ratón', categoryId: 10 } as Producto,.
  { id: 3, name: 'Libro', categoryId: 20 } as Producto
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Id de categoría: ${categoryId}`);
});
// Salida: 10, 20
```

## 🎓 Resumen

### Cuándo se debe usar distinct.
- ✅ Cuando se necesita una lista de valores únicos.
- ✅ Cuando se desea eliminar duplicados en un flujo finito
- ✅ Cuando se crean listas de identificadores o categorías

### Cuando se debe utilizar distinctUntilChanged.
- ✅ Si se desea eliminar sólo los duplicados consecutivos.
- ✅ Detección de cambios en los campos de entrada
- ✅ Cuando se desea ahorrar memoria con flujos infinitos

### Notas.
- ⚠️ Utilice el parámetro `flushes` en flujos infinitos para evitar fugas de memoria.
- ⚠️ Tenga en cuenta el uso de memoria cuando se transmiten grandes cantidades de valores únicos.
- ⚠️ Si el rendimiento es crítico, controle el tamaño del Set

## 🚀 Próximos pasos.

- **[distinctUntilChanged](./distinctUntilChanged)** - Aprende a eliminar sólo los duplicados consecutivos.
- **[distinctUntilKeyChanged](./distinctUntilKeyChanged)** - Aprende a comparar por clave de objeto.
- **[filter](./filter)** - aprende a filtrar por condiciones
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - aprende a utilizar casos de uso reales
