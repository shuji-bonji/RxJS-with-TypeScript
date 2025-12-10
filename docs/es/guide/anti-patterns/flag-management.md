---
title: Problema de proliferación de flags de gestión de estado
description: "Cómo mejorar el problema de proliferación de 17 flags booleanos en proyectos RxJS mediante diseño reactivo. Presentamos técnicas prácticas de refactorización que mejoran drásticamente la mantenibilidad y legibilidad al expresar el estado con Observables e integrar múltiples flags usando scan() y combineLatest()."
# outline: deep
---

# Proliferación de flags de gestión de estado

Incluso en proyectos que han adoptado RxJS, es común encontrar el problema de la proliferación masiva de flags booleanos dentro de componentes. Este artículo explica las causas y métodos de mejora basándose en un caso real donde existían 17 flags.

## Ejemplo del problema

Primero, veamos código real encontrado en el campo. A continuación se muestra un ejemplo típico de proliferación de flags de gestión de estado:

```typescript
class ProblematicComponent {
  // Existen 17 flags
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    // Bifurcación compleja dentro de subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Procesamiento real
              this.isSaving = false;
              this.hasUnsavedChanges = false;
            }
          }
        }
      },
      error: (err) => {
        this.isSaving = false;
        this.hasError = true;
        this.isProcessing = false;
      }
    });
  }
}
```

Este tipo de código ocurre **incluso con RxJS introducido**. Este patrón de gestionar manualmente 17 flags y controlarlos con bifurcaciones condicionales complejas tiene problemas en mantenibilidad, legibilidad y facilidad de prueba.

## ¿Por qué proliferan los flags?

Detrás de la proliferación de flags hay no solo problemas técnicos, sino también patrones de pensamiento de los desarrolladores y procesos de evolución organizacional. A continuación, analizamos cinco causas principales.

### Análisis estructural de las causas

| Categoría de causa | Síntomas específicos | Contexto |
|------------|------------|------|
| **① Persistencia del pensamiento imperativo** | Más de 10 flags como `isLoading`, `isSaving`, `isError`<br>Gran cantidad de guardas como `if (this.isSaving) return;` | Control de lógica mediante **"flags de estado" imperativos** en lugar de streams RxJS.<br>Estado y efectos secundarios no separados, disminución de legibilidad |
| **② No aprovechamiento de estados derivados** | Gestión mediante asignación directa como `this.isLoaded = true;` en el componente | Podría definirse declarativamente la derivación de estados usando `map` o `combineLatest` de Observable,<br>pero se compone estado manualmente sin hacerlo |
| **③ Responsabilidades de diseño de estado ambiguas** | Múltiples flags para el mismo estado<br>(ejemplo: `isLoadingStart`, `isLoadingEnd`) | **Tratando cambios de estado como comandos**.<br>Lo que debería integrarse como "un estado" se dispersa en múltiples flags |
| **④ Bifurcación de streams RxJS sin organizar** | Múltiples `if` y `tap` encadenados dentro de un `Observable`,<br>mezclando efectos secundarios y actualización de estado | No se logra separación de responsabilidades en diseño de streams.<br>Uso ambiguo de `switchMap` y `catchError` |
| **⑤ Ausencia de capa ViewModel** | Manipulación directa de `this.isEditing`, `this.isSaved` en componente UI | Al mantener estado dentro del componente,<br>los beneficios de RxJS se pierden |


## Causa raíz: Desajuste en el modelo de pensamiento

La causa raíz de la proliferación de flags es el **desajuste entre los modelos de pensamiento de programación imperativa y programación reactiva**. Cuando los desarrolladores usan RxJS con una mentalidad imperativa, ocurren los siguientes problemas.

### Estructura de transición

Muchos proyectos atraviesan el siguiente proceso evolutivo que lleva al infierno de flags.

```
1. Añadir control con flags if para hacerlo funcionar por el momento
   ↓
2. Introducir RxJS más tarde
   ↓
3. No se puede convertir la lógica antigua a streams, queda mezclada
   ↓
4. Se completa el infierno de flags
```

### Capas de gestión de estado mezcladas

El estado dentro de una aplicación debería gestionarse dividido en tres capas.

```
Aplicación
 ├── Estado de View (isOpen, isLoading, formDirty)     ← Dentro del componente
 ├── Estado de Business (entity, filters, errors)      ← Capa de gestión de estado
 └── Estado de API (pending, success, error)           ← RxJS stream
```

Si estas tres capas no están separadas, se mezclan **tres tipos** de "flags" con responsabilidades diferentes. Cuando se gestiona el estado de View y el estado de API al mismo nivel, la complejidad aumenta explosivamente.

## Esencia del problema: "Naturaleza" de los flags

El verdadero problema de la proliferación de flags no es "que sean muchos", sino **que los flags son variables mutables imperativas**. A continuación, comparamos flags problemáticos y flags apropiados.

### ❌ Flags problemáticos: Variables mutables imperativas

```typescript
class BadComponent {
  // Estos se han convertido en "comandos" en lugar de "estados"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Se necesita cláusula de guarda
    this.isSaving = true;              // Cambio manual

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Reinicio manual
        this.hasError = false;         // Gestión manual de otros flags también
      },
      error: () => {
        this.isSaving = false;         // Mismo procesamiento en múltiples lugares
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problemas
> - El estado es "procedimental" no "declarativo"
> - El momento de cambio de estado está disperso
> - El desarrollador debe garantizar manualmente la consistencia entre flags

### ✅ Flags apropiados: Variables reactivas

```typescript
class GoodComponent {
  // Declarar como stream de estado
  private saveAction$ = new Subject<void>();

  readonly saveState$ = this.saveAction$.pipe(
    switchMap(() =>
      this.api.save().pipe(
        map(() => 'success' as const),
        catchError(() => of('error' as const)),
        startWith('loading' as const)
      )
    ),
    startWith('idle' as const),
    shareReplay(1)
  );

  // Estados derivados también definidos declarativamente
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Solo disparo de evento
  }
}
```

> [!TIP] Mejoras
> - Estado gestionado centralizadamente como "stream"
> - Transición de estado definida declarativamente en pipeline
> - Consistencia entre flags garantizada automáticamente


## Criterios de juicio para diseño de flags

A continuación se resumen criterios para juzgar si tu código tiene un diseño de flags problemático. Úsalos como referencia durante code review o diseño.

| Aspecto | ❌ Problemático | ✅ Sin problemas |
|------|-----------|-----------|
| **Tipo** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **Método de cambio** | Asignación directa `flag = true` | Stream/derivado `state$.pipe(map(...))` |
| **Dependencias** | Implícitas (orden del código) | Explícitas (combineLatest, computed) |
| **Nomenclatura** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Cantidad** | Más de 10 booleanos independientes | 1 estado + múltiples derivados |


## Estrategia de mejora

Para resolver el problema de proliferación de flags, procede con refactorización gradual en los siguientes 3 pasos.

### Step 1: Inventario de estados

Primero, enumera todos los flags actuales y clasifícalos por responsabilidad. Esto hará visible qué flags pueden integrarse.

```typescript
// Enumerar flags existentes y clasificar responsabilidades
interface StateInventory {
  view: string[];      // Control de visualización UI (isModalOpen, isEditing)
  business: string[];  // Lógica de negocio (isFormValid, hasUnsavedChanges)
  api: string[];       // Estado de comunicación (isLoading, isSaving, hasError)
}
```

### Step 2: Conversión de estado a Enum

A continuación, integra múltiples flags booleanos relacionados en un solo estado. Por ejemplo, `isLoading`, `isSaving`, `hasError` pueden integrarse todos como "estado de solicitud".

```typescript
// Integrar múltiples booleanos en un solo estado
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Ejemplo de uso
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError ya no son necesarios
}
```

### Step 3: Hacer reactivo

Finalmente, gestiona el estado con Observable o Signal, y define los estados derivados declarativamente. Esto garantiza automáticamente la consistencia del estado.

```typescript
// Gestionar con Observable o Signal
class ReactiveComponent {
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  // Integrar como ViewModel
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$
  ]).pipe(
    map(([api, form]) => ({
      canSave: !api.saving && form.valid,
      showSpinner: api.loading || api.saving,
      showError: api.error !== null
    }))
  );
}
```


## Ejemplo de implementación: Refactorización de 17 flags

Aquí mostramos el proceso de refactorizar realmente el componente con 17 flags presentado al inicio a un diseño reactivo. Comparando Before/After, podrás experimentar el efecto de la mejora.

### Before: Gestión imperativa de flags

Primero, reconfirmemos el código problemático. 17 flags booleanos proliferan, controlados por bifurcaciones condicionales complejas.

```typescript
class LegacyComponent {
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    if (!this.isLoading &&
        !this.isSaving &&
        this.isFormValid &&
        !this.hasError &&
        this.isDataLoaded) {
      this.isSaving = true;
      this.apiService.save().subscribe({
        next: () => {
          this.isSaving = false;
          this.hasUnsavedChanges = false;
        },
        error: () => {
          this.isSaving = false;
          this.hasError = true;
        }
      });
    }
  }
}
```

### After: Gestión reactiva de estado

A continuación, veamos el código mejorado. Los 17 flags se han organizado en 3 estados básicos (apiState$, formState$, dataState$) y 1 estado derivado (vm$).

```typescript
import { BehaviorSubject, combineLatest, EMPTY } from 'rxjs';
import { map, switchMap, catchError, startWith } from 'rxjs';

interface ApiState {
  loading: boolean;
  saving: boolean;
  deleting: boolean;
  error: string | null;
}

interface DataState {
  loaded: boolean;
  editing: boolean;
}

class RefactoredComponent {
  // Gestionar estados básicos con Observable
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    deleting: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  private readonly dataState$ = new BehaviorSubject<DataState>({
    loaded: false,
    editing: false
  });

  // Integrar como ViewModel (estados derivados)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // Estados derivados para visualización UI
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Exponer estados individuales según necesidad
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // La verificación de estado la realiza automáticamente el ViewModel
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError(error => {
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: error.message
        });
        return EMPTY;
      })
    ).subscribe(() => {
      this.apiState$.next({
        ...this.apiState$.value,
        saving: false
      });
    });
  }
}
```

### Uso en el lado UI

Con la gestión reactiva de estado, el uso en el lado UI también se simplifica significativamente. Ya no es necesario verificar múltiples flags individualmente, solo obtener la información necesaria del ViewModel.

```typescript
// Before: Referencia directa a múltiples flags
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After: Obtener estado derivado del ViewModel
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Importancia de las convenciones de nomenclatura

En el diseño de flags, la nomenclatura es muy importante. Una nomenclatura apropiada permite entender de un vistazo la responsabilidad, naturaleza y ciclo de vida de ese flag. Por el contrario, la nomenclatura ambigua se convierte en fuente de confusión.

### ❌ Ejemplos de nomenclatura incorrecta

Las siguientes nomenclaturas tienen intención poco clara y reducen la mantenibilidad.

```typescript
// ¿Flag de qué? ¿Qué desencadena el cambio?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// ¿Es estado? ¿Es acción?
isProcess: boolean;  // ¿En proceso? ¿Procesado?
```

### ✅ Ejemplos de nomenclatura correcta

La nomenclatura apropiada expresa claramente la intención y naturaleza del estado. Usa Observable (sufijo `$`) o Signal, y aclara el tipo de estado (State, can, should).

```typescript
// Expresar el estado claramente
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// Los estados derivados también tienen intención clara
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Ejemplo usando Signal (disponible en Angular, Preact, Solid.js, etc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Checklist de diagnóstico

Verifica con el siguiente checklist si tu código no ha caído en el problema de proliferación de flags. Úsalo como referencia durante code review o diseño.

```markdown
## 🚨 Señales de peligro

- [ ] Hay 5 o más variables booleanas
- [ ] Hay 3 o más sentencias `if` anidadas dentro de `subscribe`
- [ ] El mismo flag se establece en múltiples lugares
- [ ] Hay 3 o más nomenclaturas tipo `isXXXing`
- [ ] Hay capa de gestión de estado pero se mantiene estado dentro del componente
- [ ] Hay múltiples nomenclaturas tipo `xxxFlag`
- [ ] El manejo de errores está disperso en cada `subscribe`

## ✅ Signos de mejora

- [ ] El estado está gestionado con `Observable` o `Signal`
- [ ] Los estados derivados están definidos con `map`/`computed`
- [ ] Las transiciones de estado están descritas declarativamente
- [ ] Se aplica el patrón ViewModel
- [ ] La nomenclatura expresa claramente la intención
```

## Resumen

Este artículo ha explicado las causas y métodos de mejora del problema de proliferación de flags en proyectos RxJS. Finalmente, revisemos los puntos importantes.

### Esencia del problema

1. **Que haya 17 flags** ← Esto es un síntoma
2. **Que sean variables mutables imperativas** ← Esto es la esencia
3. **Que las transiciones de estado no sean declarativas** ← Esto es la causa
4. **Que la nomenclatura sea ambigua (xxxFlag)** ← Esto es fuente de confusión

### Dirección de mejora

Para resolver el problema de proliferación de flags, son necesarias las siguientes 4 transformaciones.

- **Variables booleanas** → **Observable/Signal**
- **Asignación directa** → **Pipeline de streams**
- **17 independientes** → **1 estado + estados derivados**
- **xxxFlag** → **xxxState$ / canXXX$**

### Lo más importante

> [!IMPORTANT] Principio importante
> "El estado es el resultado de eventos, no se controla directamente con flags"

La introducción de RxJS es una transformación de "pensamiento", no de "sintaxis". Si arrastras el pensamiento imperativo, el infierno de flags no se resolverá. Al captar el estado como streams y diseñar declarativamente, mejoran la mantenibilidad, legibilidad y facilidad de prueba.


## Secciones relacionadas

Para profundizar el conocimiento sobre gestión de flags aprendido en este artículo, consulta también los siguientes artículos relacionados.

- [Infierno de anidamiento de if dentro de subscribe](./subscribe-if-hell) - Método apropiado de procesamiento de bifurcación condicional
- [Errores comunes y soluciones](./common-mistakes) - Detalles de 15 anti-patrones
- [Manejo de errores](/es/guide/error-handling/strategies) - Estrategias apropiadas de manejo de errores
- [Subject y Multicasting](/es/guide/subjects/what-is-subject) - Fundamentos de gestión de estado

## Recursos de referencia

Puedes aprender más profundamente con la documentación oficial de RxJS y recursos de aprendizaje.

- [Documentación oficial de RxJS](https://rxjs.dev/) - Referencia de API y guías oficiales
- [Learn RxJS](https://www.learnrxjs.io/) - Ejemplos prácticos por operador
- [RxJS Marbles](https://rxmarbles.com/) - Entender visualmente el comportamiento de operadores
