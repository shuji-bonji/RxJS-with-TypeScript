---
description: "Se explicarán las Creation Function que generan valores en forma de bucle: aprende a usar range y generate para implementar procesos iterativos como las sentencias for y while como flujos Observable. Desde la generación de números secuenciales hasta complejas transiciones de estado basadas en condiciones personalizadas, puedes realizar un procesamiento de bucle declarativo que haga uso de la inferencia de tipos de TypeScript."
---

# Sistema de generación de bucles Creation Function

Creation Function para representar el procesamiento de bucles como las sentencias for y while como Observable.

## Sistema de generación de bucles ¿Qué es Creation Function?

Creation Function para sistemas de generación de bucles realiza de forma reactiva procesos repetitivos. Al sustituir los bucles imperativos tradicionales (sentencias `for` y `while`) por flujos Observable declarativos, permiten un procesamiento flexible en combinación con la cadena de operadores de RxJS.

Consulte la siguiente tabla para ver las características y el uso de cada Creation Function.

## Principales sistemas de generación de bucles Creation Function

| Función | Descripción | Caso de uso. |
|---|---|---|
| **[range](/es/guide/funciones-creación/loop/range)** | Generar un rango de números (para sentencias) | Generación de números secuenciales, procesamiento por lotes. |
| **[generate](/es/guide/función de creación/lazo/generate)** | Generación de bucles genéricos (tipo while) | Repetición condicional, transiciones de estado complejas |

## Criterios de utilización

La elección de las Creation Function generadoras de bucles viene determinada por los siguientes aspectos.

### 1. patrón de generación

- **Secuencia numérica**: `range()` - generación secuencial simple de números con valores inicial y final.
- **Condiciones complejas**: `generate()` - control libre sobre valores iniciales, condiciones, iteraciones y selección de resultados.

### 2. Tipos de bucles

- Bucle tipo instrucción for**: `range()` - `for (let i = start; i <= end; i++)`
- bucles tipo while**: `generate()` - `while (condition) { ... }`

### 3. flexibilidad

- **Suficientemente simple**: `range()` - si se requiere una secuencia de números
- **Necesita control avanzado**: `generate()` - gestión personalizada de estados, bifurcación condicional, control de pasos

## Casos prácticos

### range() - generación secuencial de números

Para la generación de números secuenciales simples, `range()` es la mejor opción.

```typescript
import { range, map } from 'rxjs';
// 1a5Generar números secuenciales a partir de
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Utilizar en procesamiento por lotes
range(0, 10).pipe(
  map(i => `Procesamiento${i + 1}`)
).subscribe(console.log);
// Salida: Procesamiento1, Procesamiento2, ..., Procesamiento10
```

### generate() - bucle condicional

Utilice `generate()` cuando se requieran condiciones complejas o una gestión de estados personalizada.

```typescript
import { generate } from 'rxjs';

// Generar secuencia Fibonacci (primer10término)
generate(
  { current: 0, next: 1, count: 0 },  // Condición inicial
  state => state.count < 10,           // Condición de continuación
  state => ({                          // Actualización del estado
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Selección del resultado
).subscribe(console.log);
// Salida: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparado con el bucle imperativo

Esta es una comparación entre los bucles imperativos convencionales y el sistema de generación de bucles RxJS Creation Function.

### Sentencia imperativa for

```typescript
// ConvencionalforSentencia
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Declarativo range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJSderange()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

```typescript
import { range, map } from 'rxjs';
// 1a5Generar números secuenciales a partir de
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Utilizar en procesamiento por lotes
range(0, 10).pipe(
  map(i => `Procesamiento${i + 1}`)
).subscribe(console.log);
// Salida: Procesamiento1, Procesamiento2, ..., Procesamiento10
```

> **Beneficios del enfoque declarativo**:.
> - Mejora de la legibilidad gracias al procesamiento en cadena.
> - Manejo uniforme de errores
> - Fácil de combinar con el procesamiento asíncrono
> - Facilidad para cancelar y abortar (por ejemplo, `takeUntil()`)

## Conversión de frío a caliente

Como se muestra en la tabla anterior, **todas las Creation Function que generan bucles generan un Cold Observable. Cada suscripción inicia una ejecución independiente.

Sin embargo, **los Observable Fríos pueden convertirse** en Observable Calientes utilizando operadores de multidifusión (`share()`, `shareReplay()`, etc.).

### Ejemplo práctico: compartir los resultados de un cálculo.


```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Cálculo independiente por abono
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Cálculo en curso:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Abonado1:', val));
cold$.subscribe(val => console.log('Abonado2:', val));
// → Un cálculo se2realiza una vez (en2000Cálculo realizado una vez)

// 🔥 Hot - Los resultados del cálculo se comparten entre abonados
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Cálculo en curso:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Abonado1:', val));
hot$.subscribe(val => console.log('Abonado2:', val));
// → Un cálculo se1Sólo se ejecuta una vez (1000Cálculo realizado una vez)
```

```typescript
import { range, map } from 'rxjs';
// 1a5Generar números secuenciales a partir de
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Utilizar en procesamiento por lotes
range(0, 10).pipe(
  map(i => `Procesamiento${i + 1}`)
).subscribe(console.log);
// Salida: Procesamiento1, Procesamiento2, ..., Procesamiento10
```

> **Casos en los que se requiere Hotting**:.
> - Los cálculos de alto coste se utilizan en múltiples ubicaciones
> - Resultados del procesamiento por lotes compartidos por múltiples componentes
> - Resultados del proceso de paginación mostrados en múltiples componentes de la UI.
>
> Para obtener más información, consulte [Sistemas básicos de creación - Conversión de frío a caliente](/es/guide/creation-functions/basic/#cold- to -hot-).

## Combinado con procesamiento asíncrono

El sistema de generación de bucles Creation Function puede combinarse con el procesamiento asíncrono para proporcionar una potente funcionalidad.

### Ejecución secuencial de llamadas a la API


```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Función para simular la adquisición de datos de la página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Datos${page}-1`, `Datos${page}-2`, `Datos${page}-3`]
  }).pipe(
    delay(300) // APISimular una llamada a
  );
}

// Página1a10Recuperar secuencialmente hasta (con un retardo de1segundos de retraso entre cada solicitud)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe({
  next: data => console.log(`Página ${data.page} Obtención:`, data.items),
  error: err => console.error('Error:', err)
});
```

### Uso en el procesamiento de reintentos

```typescript
import { range, throwError, of, Observable, mergeMap, retry, delay } from 'rxjs';
// Función para simular la obtención de datos (falla aleatoriamente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40%Éxito con una probabilidad de

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Fallo en la adquisición de datos'))
        : of('Adquisición de datos con éxito')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    // RxJS 7.3+ Recomendación: retry({ count, delay }) Formato
    retry({
      count: 3, // Máx.3Reintentos
      delay: (error, retryCount) => {
        console.log(`Reintentos ${retryCount}/3`);
        // Retroceso exponencial: 1Segundos2Segundos4Segundos
        return range(0, 1).pipe(delay(Math.pow(2, retryCount - 1) * 1000));
      }
    })
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultado:', result),
  error: err => console.error('Error:', err.message)
});

// Ejemplo de salida:
// Reintentos 1/3
// Reintentos 2/3
// Resultado: Adquisición de datos con éxito
```

## Relación con Pipeable Operator

Las Creation Function que generan bucles no tienen un Pipeable Operator homólogo directo. Siempre se utilizan como Creation Function.

Sin embargo, pueden combinarse con los siguientes operadores para un procesamiento más avanzado.

```typescript
import { range, map } from 'rxjs';
// 1a5Generar números secuenciales a partir de
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Utilizar en procesamiento por lotes
range(0, 10).pipe(
  map(i => `Procesamiento${i + 1}`)
).subscribe(console.log);
// Salida: Procesamiento1, Procesamiento2, ..., Procesamiento10
```

## Notas de rendimiento.

Las Creation Function que generan bucles emiten valores de forma sincrónica, por lo que hay que tener en cuenta el rendimiento cuando se generan grandes cantidades de valores.

> [!WARNING]

> **Manejo de grandes cantidades de datos**:.
> - Grandes cantidades de datos, como `range(1, 1000000)`, se emiten todos de forma sincrónica y consumen memoria
> - Buffer con `bufferCount()` o `windowCount()` según sea necesario
> - o cambiar a ejecución asíncrona especificando un planificador con `scheduled()`.


```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Ejecutado por el programador asíncrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Próximos pasos.

Para obtener más información sobre el funcionamiento de cada Creation Function y ejemplos prácticos, haga clic en los enlaces de la tabla anterior.

También puede obtener más información sobre [Funciones de creación básicas](/es/guide/creation-functions/basic/), [Funciones de creación combinadas](/es/guide/creation-functions/combination/), [Selección y Creation Function](/es/guide/creation-functions/selection/) y [Creation Function](/es/guide/creation-functions/conditional/). Esto le ayudará a comprender el conjunto de las Creation Function.
