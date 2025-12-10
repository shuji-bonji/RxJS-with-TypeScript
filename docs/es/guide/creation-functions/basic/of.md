---
description: "of() - La Función de Creación más simple que emite valores especificados en secuencia: Perfecta para datos de prueba, mocks, valores predeterminados y ramificación condicional"
---

# of() - Emisión Secuencial de Valores

`of()` es la Función de Creación más simple que emite los valores especificados uno por uno en secuencia.

## Resumen

`of()` emite los valores pasados como argumentos en secuencia a medida que se suscribe, y se completa inmediatamente después de emitir todos los valores. Se usa frecuentemente para crear código de prueba o datos mock.

**Firma**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Documentación Oficial**: [📘 RxJS Oficial: of()](https://rxjs.dev/api/index/function/of)

## Uso Básico

`of()` permite pasar múltiples valores separados por comas.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completado')
});

// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Valor: 4
// Valor: 5
// Completado
```

## Características Importantes

### 1. Emisión Síncrona

`of()` emite todos los valores **síncronamente** al suscribirse.

```typescript
import { of } from 'rxjs';

console.log('Antes de la suscripción');

of('A', 'B', 'C').subscribe(value => console.log('Valor:', value));

console.log('Después de la suscripción');

// Salida:
// Antes de la suscripción
// Valor: A
// Valor: B
// Valor: C
// Después de la suscripción
```

### 2. Completación Inmediata

Notifica `complete` inmediatamente después de emitir todos los valores.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('¡Completado!')
});

// Salida: 1, 2, 3, ¡Completado!
```

### 3. Puede Emitir Cualquier Tipo de Valor

Se pueden emitir valores de cualquier tipo, desde tipos primitivos hasta objetos y arrays.

```typescript
import { of } from 'rxjs';

// Tipos primitivos
of(42, 'hola', true).subscribe(console.log);

// Objetos
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Arrays (emite el array en sí como un único valor)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Salida: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` es un **Cold Observable**. Cada suscripción inicia una ejecución independiente.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Primera suscripción
values$.subscribe(val => console.log('Suscriptor A:', val));

// Segunda suscripción (ejecutada independientemente)
values$.subscribe(val => console.log('Suscriptor B:', val));

// Salida:
// Suscriptor A: 1
// Suscriptor A: 2
// Suscriptor A: 3
// Suscriptor B: 1
// Suscriptor B: 2
// Suscriptor B: 3
```

> [!NOTE]
> **Características de Cold Observable**:
> - Se inicia una ejecución independiente para cada suscripción
> - Cada suscriptor recibe su propio flujo de datos
> - Si necesitas compartir datos, debes hacerlo Hot con `share()` etc.
>
> Ver [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables) para más información.

## Diferencia Entre of() y from()

`of()` y `from()` tienen comportamientos diferentes al tratar con arrays. Este es un punto común de confusión.

```typescript
import { of, from } from 'rxjs';

// of() - emite el array como un único valor
of([1, 2, 3]).subscribe(console.log);
// Salida: [1, 2, 3]

// from() - emite cada elemento del array individualmente
from([1, 2, 3]).subscribe(console.log);
// Salida: 1, 2, 3
```

> [!IMPORTANT]
> **Criterios de Uso**:
> - Para emitir el array en sí → `of([1, 2, 3])`
> - Para emitir cada elemento de un array por separado → `from([1, 2, 3])`

## Casos de Uso Prácticos

### 1. Datos de Prueba y Creación de Mocks

`of()` se usa más frecuentemente para crear datos mock en código de prueba.

```typescript
import { of } from 'rxjs';

// Datos de usuario mock
function getMockUser$() {
  return of({
    id: 1,
    name: 'Usuario de Prueba',
    email: 'test@example.com'
  });
}

// Uso en pruebas
getMockUser$().subscribe(user => {
  console.log('Usuario:', user.name); // Usuario: Usuario de Prueba
});
```

### 2. Proporcionar Valores Predeterminados

Se usa para proporcionar valores de respaldo en caso de errores o valores predeterminados.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('ID inválido'));
  }
  return of({ id, data: 'algunos datos' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Error:', err.message);
    return of({ id: 0, data: 'datos predeterminados' }); // Valor predeterminado
  })
).subscribe(result => console.log(result));
// Salida: Error: ID inválido
//         { id: 0, data: 'datos predeterminados' }
```

### 3. Emitir Múltiples Valores Gradualmente

Se usa para ejecutar múltiples pasos en secuencia.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Cargando...', 'Procesando...', '¡Listo!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Salida (cada 1 segundo):
// Cargando...
// Procesando...
// ¡Listo!
```

### 4. Valores de Retorno en Ramificación Condicional

Se usa en combinación con `iif()` y `switchMap()` para retornar valores según condiciones.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('¡Bienvenido de vuelta!'),
  of('Por favor inicia sesión')
).subscribe(console.log);
// Salida: ¡Bienvenido de vuelta!
```

## Uso en Pipeline

`of()` se usa como punto de inicio de un pipeline o para inyectar datos en el camino.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Solo números pares
  map(n => n * 10)           // Multiplicar por 10
).subscribe(console.log);
// Salida: 20, 40
```

## Errores Comunes

### 1. Pasar un Array Directamente

```typescript
// ❌ Incorrecto - el array completo se emite como un único valor
of([1, 2, 3]).subscribe(console.log);
// Salida: [1, 2, 3]

// ✅ Correcto - usar from() para emitir cada elemento por separado
from([1, 2, 3]).subscribe(console.log);
// Salida: 1, 2, 3

// ✅ O usar sintaxis spread
of(...[1, 2, 3]).subscribe(console.log);
// Salida: 1, 2, 3
```

### 2. Confusión con Procesamiento Asíncrono

Ten en cuenta que `of()` emite síncronamente. No es procesamiento asíncrono.

```typescript
// ❌ Esto no se vuelve asíncrono
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() se ejecuta inmediatamente y el objeto Promise se emite

// ✅ Usar from() para transmitir una Promise
from(fetchDataFromAPI()).subscribe(console.log);
```

## Consideraciones de Rendimiento

`of()` es muy ligero y tiene poca sobrecarga de rendimiento. Sin embargo, al emitir grandes cantidades de valores, ten en cuenta lo siguiente.

> [!TIP]
> Al emitir un gran número de valores (miles o más) secuencialmente, considera usar `from()` o `range()`.

## Funciones de Creación Relacionadas

| Función | Diferencia | Uso |
|----------|------|----------|
| **[from()](/es/guide/creation-functions/basic/from)** | Convertir desde array o Promise | Transmitir iterables o Promises |
| **range()** | Generar un rango de números | Emitir números consecutivos |
| **EMPTY** | Completar inmediatamente sin emitir nada | Cuando se necesita un stream vacío |

## Resumen

- `of()` es la Función de Creación más simple que emite los valores especificados en secuencia
- Se emite síncronamente al suscribirse y se completa instantáneamente
- Ideal para datos de prueba y creación de mocks
- Si se pasa un array, se emite el array en sí (diferente de `from()`)
- Usar `from()` para procesamiento asíncrono

## Próximos Pasos

- [from() - Convertir desde Array, Promise, etc.](/es/guide/creation-functions/basic/from)
- [Funciones de Creación de Combinación](/es/guide/creation-functions/combination/)
- [Volver a Funciones de Creación Básicas](/es/guide/creation-functions/basic/)
