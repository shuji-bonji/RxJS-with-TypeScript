---
description: Esta sección describe las Funciones de Creación que generan valores de manera similar a bucles, usando range y generate para aprender cómo implementar procesamiento iterativo como sentencias for y while como flujos Observable. Desde generación de números secuenciales hasta transiciones de estado complejas basadas en condiciones personalizadas, puedes realizar procesamiento de bucles declarativo utilizando la inferencia de tipos de TypeScript.
---

# Funciones de Creación de Generación de Bucles

Funciones de Creación para expresar procesamiento de bucles como sentencias for y while como Observable.

## ¿Qué son las Funciones de Creación de Generación de Bucles?

Las Funciones de Creación de Generación de Bucles realizan procesamiento repetitivo de manera reactiva. Al reemplazar los bucles imperativos convencionales (sentencias `for` y `while`) con flujos Observable declarativos, es posible un procesamiento flexible en combinación con la cadena de operadores de RxJS.

Revisa la tabla a continuación para ver las características y uso de cada Función de Creación.

## Principales Funciones de Creación de Generación de Bucles

| Función | Descripción | Casos de Uso |
|----------|------|-------------|
| **[range](/es/guide/creation-functions/loop/range)** | Generar un rango de números (como sentencia for) | Generación de números secuenciales, procesamiento por lotes |
| **[generate](/es/guide/creation-functions/loop/generate)** | Generación de bucles de propósito general (como sentencia while) | Repetición condicional, transiciones de estado complejas |

## Criterios de Uso

La selección de Funciones de Creación de Generación de Bucles se determina desde las siguientes perspectivas.

### 1. Patrón de Generación

- **Secuencia numérica**: `range()` - Generación simple de números secuenciales con valores de inicio y fin
- **Condiciones complejas**: `generate()` - Control libre sobre valores iniciales, condiciones, iteración y selección de resultados

### 2. Tipos de Bucle

- **Bucle tipo sentencia for**: `range()` - `for (let i = start; i <= end; i++)`
- **Bucle tipo sentencia while**: `generate()` - `while (condition) { ... }`

### 3. Flexibilidad

- **Simple es suficiente**: `range()` - Cuando necesitas una secuencia de números
- **Necesita control avanzado**: `generate()` - Gestión de estado personalizada, ramificación condicional, control de pasos

## Ejemplos de Uso Práctico

### range() - Generación de Números Secuenciales

Para generación simple de números secuenciales, `range()` es la mejor opción.

```typescript
import { range, map } from 'rxjs';
// Generar números secuenciales del 1 al 5
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Usar en procesamiento por lotes
range(0, 10).pipe(
  map(i => `Proceso ${i + 1}`)
).subscribe(console.log);
// Salida: Proceso 1, Proceso 2, ..., Proceso 10
```

### generate() - Bucle Condicional

Usa `generate()` para condiciones complejas o gestión de estado personalizada.

```typescript
import { generate } from 'rxjs';

// Generar secuencia de Fibonacci (primeros 10 términos)
generate(
  { current: 0, next: 1, count: 0 },  // Estado inicial
  state => state.count < 10,           // Condición de continuación
  state => ({                          // Actualización de estado
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Selector de resultado
).subscribe(console.log);
// Salida: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparación con Bucle Imperativo

Esta es una comparación entre el bucle imperativo convencional y las Funciones de Creación de Generación de Bucles de RxJS.

### Sentencia for Imperativa

```typescript
// Sentencia for convencional
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### range() Declarativo

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Ventajas del enfoque declarativo**:
> - Mejor legibilidad con procesamiento de pipeline
> - Manejo de errores uniforme
> - Fácil de combinar con procesamiento asíncrono
> - Fácil de cancelar y abortar (ej., `takeUntil()`)

## Conversión de Cold a Hot

Como se muestra en la tabla anterior, **todas las Funciones de Creación de Generación de Bucles generan Cold Observables**. Cada suscripción inicia una ejecución independiente.

Sin embargo, usando operadores de multicast (`share()`, `shareReplay()`, etc.), puedes **convertir un Cold Observable a un Hot Observable**.

### Ejemplo Práctico: Compartir Resultados de Cálculo

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Cálculo independiente para cada suscripción
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculando:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Suscriptor 1:', val));
cold$.subscribe(val => console.log('Suscriptor 2:', val));
// → Cálculo ejecutado dos veces (2000 cálculos)

// 🔥 Hot - Compartir resultados de cálculo entre suscriptores
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculando:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Suscriptor 1:', val));
hot$.subscribe(val => console.log('Suscriptor 2:', val));
// → Cálculo ejecutado solo una vez (1000 cálculos)
```

> [!TIP]
> **Casos donde se requiere conversión a Hot**:
> - Usar cálculos de alto costo en múltiples ubicaciones
> - Compartir resultados de procesamiento por lotes con múltiples componentes
> - Mostrar resultados de paginación en múltiples componentes de UI
>
> Para más información, ver [Creación Básica - Conversión de Cold a Hot](/es/guide/creation-functions/basic/#conversion-de-cold-a-hot).

## Combinación con Procesamiento Asíncrono

Las Funciones de Creación de Generación de Bucles demuestran funcionalidad poderosa cuando se combinan con procesamiento asíncrono.

### Ejecución Secuencial de Llamadas API

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Función para simular obtención de datos de página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Data${page}-1`, `Data${page}-2`, `Data${page}-3`]
  }).pipe(
    delay(300) // Simular llamada API
  );
}

// Obtener páginas 1 a 10 secuencialmente (con 1 segundo de retraso entre cada solicitud)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Página ${data.page} obtenida:`, data.items),
  err => console.error('Error:', err)
);
```

### Uso en Procesamiento de Reintentos

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Función para simular obtención de datos (falla aleatoriamente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40% tasa de éxito

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Obtención de datos fallida'))
        : of('Obtención de datos exitosa')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Reintentar hasta 3 veces
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Reintento ${index + 1}/3`);
          // Backoff exponencial: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultado:', result),
  error: err => console.error('Error:', err.message)
});

// Ejemplo de salida:
// Reintento 1/3
// Reintento 2/3
// Resultado: Obtención de datos exitosa
```

## Relación con Pipeable Operator

Las Funciones de Creación de Generación de Bucles no tienen un Pipeable Operator correspondiente directo. Siempre se usan como Funciones de Creación.

Sin embargo, es posible un procesamiento más avanzado combinándolas con los siguientes operadores:

| Operadores a Combinar | Propósito |
|-------------------|------|
| `map()` | Transformar cada valor |
| `filter()` | Pasar solo valores que coincidan con la condición |
| `take()`, `skip()` | Controlar el número de valores |
| `concatMap()`, `mergeMap()` | Ejecutar procesamiento asíncrono para cada valor |
| `toArray()` | Recopilar todos los valores en un array |

## Notas de Rendimiento

Las Funciones de Creación de Generación de Bucles emiten valores síncronamente, así que ten cuidado con el rendimiento al generar un gran número de valores.

> [!WARNING]
> **Manejo de grandes cantidades de datos**:
> - Grandes cantidades de datos, como `range(1, 1000000)`, se emiten todas síncronamente y consumen memoria
> - Usar buffer con `bufferCount()` o `windowCount()` según sea necesario
> - O cambiar a ejecución asíncrona especificando un scheduler con `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Ejecutar con scheduler asíncrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Próximos Pasos

Para aprender más sobre el comportamiento detallado y ejemplos prácticos de cada Función de Creación, haz clic en los enlaces de la tabla anterior.

También puedes entender el panorama completo de las Funciones de Creación aprendiendo [Funciones de Creación Básicas](/es/guide/creation-functions/basic/), [Funciones de Creación de Combinación](/es/guide/creation-functions/combination/), [Funciones de Creación de Selección/Partición](/es/guide/creation-functions/selection/) y [Funciones de Creación Condicionales](/es/guide/creation-functions/conditional/).
