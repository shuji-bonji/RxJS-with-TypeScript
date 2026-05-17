---
description: El operador filter es un operador básico de RxJS que filtra valores según condiciones especificadas y se utiliza para controlar flujos de datos. Al igual que Array.prototype.filter(), utiliza una función de predicado para determinar qué valores dejar pasar, permitiendo la selección condicional de valores y el filtrado con seguridad de tipos.
---

# filter - Filtrar Valores Basándose en Condiciones

El operador `filter` pasa solo valores que **satisfacen una condición especificada** (función de predicado).
Este es el mismo concepto que `Array.prototype.filter()` de JavaScript aplicado a Observables.

## 🔰 Sintaxis Básica y Uso

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

source$.pipe(
  filter(x => x % 2 === 0) // Pasar solo números pares
).subscribe(value => {
  console.log('Valor:', value);
});

// Salida:
// Valor: 2
// Valor: 4
// Valor: 6
// Valor: 8
// Valor: 10
```

- La función de predicado `(value) => boolean` determina qué valores dejar pasar.
- Solo los valores que devuelven `true` se pasan al siguiente operador.

## 💡 Patrones de Uso Típicos

- **Filtrado de datos**: Seleccionar solo valores que satisfagan condiciones específicas
- **Validación de entrada**: Permitir solo valores válidos
- **Procesamiento condicional**: Procesar diferentes flujos según condiciones específicas
- **Guardia de tipo**: Restringir tipos de TypeScript

## 🧠 Ejemplo de Código Práctico: Validación de Entrada de Usuario

Este ejemplo filtra valores de entrada para permitir solo caracteres numéricos.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Crear campo de entrada
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Ingrese solo números...';
input.style.padding = '8px';
input.style.margin = '10px';
document.body.appendChild(input);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
document.body.appendChild(output);

// Evento de entrada
const input$ = fromEvent<InputEvent>(input, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  filter(value => /^\d*$/.test(value)) // Permitir solo caracteres numéricos
);

input$.subscribe(value => {
  output.textContent = `Valor válido: ${value}`;
  console.log('Valor numérico:', value);
});

// Si se ingresan caracteres no numéricos, el evento se filtra
```

## 🔍 Diferencia con buffer

| Operador | Comportamiento | Salida |
|:---|:---|:---|
| `filter` | Descarta valores que **no coinciden** con la condición | Valor individual `T` |
| `buffer` | **Acumula** valores en un array | Array `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Pasar solo valores que coincidan con la condición
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Salida: filter: 0
  // Salida: filter: 2
  // Salida: filter: 4
});

// buffer - Acumular valores como un array
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Salida: buffer: [0, 1]
  // Salida: buffer: [2, 3, 4]
});
```

[🌐 Documentación Oficial de RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## ⚠️ Notas

### 1. Las Funciones de Predicado Deben Ser Funciones Puras

Evita funciones de predicado con efectos secundarios.

```ts
// ❌ Mal ejemplo: Con efectos secundarios
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Efecto secundario
    return x > 10;
  })
).subscribe();

// ✅ Buen ejemplo: Función pura
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. Uso de Funciones de Guardia de Tipo

Puedes aprovechar la seguridad de tipos de TypeScript.

```ts
interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Usar como función de guardia de tipo
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email se infiere como tipo string
});
```

## 📚 Operadores Relacionados

- [take](/es/guide/operators/filtering/take) - Obtener solo los primeros N valores
- [first](/es/guide/operators/filtering/first) - Obtener solo el primer valor (condicionalmente posible)
- [distinct](/es/guide/operators/filtering/distinct) - Excluir valores duplicados
- [distinctUntilChanged](/es/guide/operators/filtering/distinctUntilChanged) - Excluir valores que son iguales al anterior

## Resumen

El operador `filter` es la herramienta de filtrado más básica en RxJS.

- ✅ Pasa solo valores que coinciden con la condición
- ✅ Se puede usar de la misma manera que `.filter()` para arrays
- ✅ Se puede usar como guardia de tipo de TypeScript
- ⚠️ Las funciones de predicado deben ser funciones puras

> [!WARNING] Atención en código de producción
> El ejemplo anterior omite la cancelación de suscripción de `fromEvent` para simplificar la explicación. En código real, gestione explícitamente el ciclo de vida con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Detalles: [Superar dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)
