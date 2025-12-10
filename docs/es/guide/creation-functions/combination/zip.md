---
description: La Función de Creación zip alinea y empareja los valores en el orden correspondiente de múltiples Observables y los emite al mismo tiempo que todas las fuentes han emitido sus valores uno por uno.
---

# zip - emparejar valores correspondientes

`zip` es una Función de Creación que agrupa **valores de orden correspondiente** emitidos desde múltiples Observables y los emite como un array o tupla.
Espera a que lleguen valores de todos los Observables fuente, uno a la vez, y crea pares cuando están listos.


## Sintaxis básica y uso

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Salida:
// A 0
// B 10
// C 20
```

- Cuando cada Observable emite un valor a la vez, se crea y emite un par.
- Si uno se retrasa, esperará hasta que ambos estén alineados.

[🌐 Documentación Oficial RxJS - `zip`](https://rxjs.dev/api/index/function/zip)


## Patrones de uso típicos

- **Mapear solicitudes a respuestas**
- **Emparejar sincrónicamente IDs con datos correspondientes**
- **Combinar múltiples flujos procesados en paralelo en un conjunto**


## Ejemplos de código práctico (con UI)

Ejemplo de **combinar y mostrar** diferentes fuentes de datos (fruta y precio).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de zip:</h3>';
document.body.appendChild(output);

// Flujo de nombres de frutas
const fruits$ = of('🍎 Manzana', '🍌 Banana', '🍇 Uva');

// Flujo de precios (emitido cada 2 segundos)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip y mostrar
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - €${price}`;
  output.appendChild(item);
});
```

- Las listas de frutas y precios se emparejan y muestran **cuando** están alineadas en correspondencia uno a uno.
- Si falta alguno, no se emitirá en ese momento.


## Operadores Relacionados

- **[zipWith](/es/guide/operators/combination/zipWith)** - Versión Pipeable Operator (usado en pipeline)
- **[combineLatest](/es/guide/creation-functions/combination/combineLatest)** - Función de Creación que combina los valores más recientes
- **[withLatestFrom](/es/guide/operators/combination/withLatestFrom)** - solo el flujo principal dispara
