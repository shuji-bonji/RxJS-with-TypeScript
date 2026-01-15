---
description: concatWith es un operador de combinación de RxJS que une otros Observables en secuencia después de la completación del Observable original. Es ideal para procesamiento secuencial dentro de un pipeline, procesamiento de seguimiento después de la completación, carga de datos por etapas y otras situaciones donde se desea agregar procesamiento posterior como una extensión del stream principal. La versión Pipeable Operator es conveniente para su uso dentro de un pipeline.
titleTemplate: ':title'
---

# concatWith - Concatenar en Secuencia

El operador `concatWith` **concatena secuencialmente** los otros Observables especificados después de que el Observable original `completa`.
Esta es la versión Pipeable Operator de la Creation Function `concat`.

## 🔰 Sintaxis Básica y Uso

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Salida: A → B → C → D → E → F
```

- Después de que `obs1$` completa, `obs2$` inicia, y después de que `obs2$` completa, `obs3$` inicia.
- Se puede usar dentro de cadenas `.pipe()`, facilitando la combinación con otros operadores.

[🌐 Documentación Oficial de RxJS - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Patrones de Uso Típicos

- **Procesamiento secuencial dentro de un pipeline**: Combinar datos adicionales en secuencia al stream transformado
- **Procesamiento de seguimiento después de la completación**: Agregar limpieza y notificaciones después de que el procesamiento principal completa
- **Carga de datos por etapas**: Adquirir datos adicionales secuencialmente después de la adquisición de datos iniciales


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de mostrar elementos recomendados relacionados en orden después de mostrar los resultados de búsqueda principales.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo Práctico de concatWith:</h3>';
document.body.appendChild(output);

// Resultados de búsqueda principales
const searchResults$ = of('🔍 Resultado de Búsqueda 1', '🔍 Resultado de Búsqueda 2', '🔍 Resultado de Búsqueda 3').pipe(
  delay(500)
);

// Elementos recomendados 1
const recommendations1$ = of('💡 Producto Recomendado A', '💡 Producto Recomendado B').pipe(
  delay(300)
);

// Elementos recomendados 2
const recommendations2$ = of('⭐ Producto Popular X', '⭐ Producto Popular Y').pipe(
  delay(300)
);

// Combinar en secuencia y mostrar
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- Los resultados de búsqueda se muestran primero,
- Luego los productos recomendados se muestran en orden.
- Se puede usar en combinación con otros operadores como `map` dentro del pipeline.


## 🔄 Diferencia con la Creation Function `concat`

### Diferencias Básicas

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Ubicación de Uso** | Usado como función independiente | Usado dentro de cadena `.pipe()` |
| **Sintaxis** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Primer Stream** | Trata todos por igual | Trata como stream principal |
| **Ventaja** | Simple y legible | Fácil de combinar con otros operadores |

### Ejemplos de Uso Específicos

**Creation Function Recomendada Solo para Combinación Simple**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Simple y legible
concat(part1$, part2$, part3$).subscribe(console.log);
// Salida: A → B → C → D → E → F
```

**Pipeable Operator Recomendado si Se Necesita Transformación**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Versión Creation Function - se vuelve verbosa
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Versión Pipeable Operator - completada en un pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // Solo 30 años o más
    map(user => user.name),          // Extraer solo nombre
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Salida: Alice → Charlie
```

**Cuando Se Agrega Procesamiento Posterior al Stream Principal**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, mapTo } from 'rxjs';

// Crear botón y área de salida
const button = document.createElement('button');
button.textContent = 'Haz clic 3 veces';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Versión Pipeable Operator - natural como extensión del stream principal
clicks$
  .pipe(
    take(3),                          // Obtener primeros 3 clics
    mapTo('Clicado'),
    concatWith(of('Completado'))      // Agregar mensaje después de completación
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Escribir el mismo comportamiento con versión Creation Function...
// ❌ Versión Creation Function - necesita separar stream principal
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    mapTo('Clicado')
  ),
  of('Completado')
).subscribe(console.log);
```

### Resumen

- **`concat`**: Óptimo para simplemente combinar múltiples streams
- **`concatWith`**: Óptimo cuando se desea agregar procesamiento posterior al stream principal mientras se transforma o procesa


## ⚠️ Notas Importantes

### Retraso Debido a Esperar la Completación

El siguiente Observable no iniciará hasta que el Observable original complete.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // Completar con 3 valores
  concatWith(of('Completo'))
).subscribe(console.log);
// Salida: 0 → 1 → 2 → Completo
```

### Manejo de Errores

Si ocurre un error en el Observable anterior, los Observables posteriores no se ejecutarán.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Ocurrió un error'))
  .pipe(
    catchError(err => of('Error recuperado')),
    concatWith(of('Siguiente proceso'))
  )
  .subscribe(console.log);
// Salida: Error recuperado → Siguiente proceso
```


## 📚 Operadores Relacionados

- **[concat](/es/guide/creation-functions/combination/concat)** - Versión Creation Function
- **[mergeWith](/es/guide/operators/combination/mergeWith)** - Versión Pipeable para combinación paralela
- **[concatMap](/es/guide/operators/transformation/concatMap)** - Mapear cada valor secuencialmente
