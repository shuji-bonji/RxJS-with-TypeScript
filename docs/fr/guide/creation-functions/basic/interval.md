---
description: "interval() - Creation Function qui émet continuellement des valeurs (nombres séquentiels à partir de 0) à des intervalles spécifiés, idéal pour le polling, l'exécution périodique et le contrôle d'animation ; différences avec timer(), comment limiter avec take(), implémentation type-safe en TypeScript, fuites de mémoire. Explique le modèle de désabonnement pour éviter les fuites de mémoire."
---

# interval() - émission continue à des intervalles spécifiés

interval()` est une Creation Function qui émet en continu des valeurs à des intervalles de temps spécifiés.

## Vue d'ensemble.

`interval()` émet continuellement des valeurs continues à partir de 0 à des intervalles de millisecondes spécifiés. Elle est fréquemment utilisée pour les processus d'interrogation et l'exécution de tâches périodiques.

**Signature** :.

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Documentation officielle** : [📘 RxJS formula : interval()](https://rxjs.dev/api/index/function/interval)

## Utilisation de base

`interval()` émet un nombre qui compte à rebours à un intervalle spécifié.

```typescript
import { interval } from 'rxjs';

// 1Valeur émise chaque seconde
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Valeur:', value);
});

// Sortie (en1(chaque seconde):
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Valeur: 3
// ...(continue indéfiniment)
```

## Caractéristiques importantes.

### 1. nombres consécutifs à partir de 0

`interval()` émet un nombre entier qui commence toujours à 0 et s'incrémente de 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Première5Seules les deux premières valeurs sont extraites
).subscribe(value => console.log(value));

// Sortie (en500ms(par seconde):
// 0
// 1
// 2
// 3
// 4
```

### 2. non terminé (flux infini)

`interval()` ne se termine pas automatiquement et **doit être désinscrit**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Valeur:', value);
});

// 5Désabonnement après 2 secondes
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Désabonné');
}, 5000);
```

> [!WARNING]

**Fuite de mémoire si vous oubliez de vous désinscrire**
>
> > Puisque `interval()` continue d'émettre des valeurs indéfiniment, oublier de se désabonner peut causer des fuites de mémoire et des problèmes de performance. Assurez-vous d'appeler `unsubscribe()` ou d'utiliser des opérateurs tels que `take()`, `takeUntil()` ou `takeWhile()` pour terminer automatiquement.

### 4. Observable à froid.

`interval()` est un Cold Observable, qui crée un timer indépendant pour chaque abonnement.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Abonné1
interval$.subscribe(value => console.log('Observer 1:', value));

// 2Abonné après quelques secondes2Ajouté
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Sortie:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← Sur minuterie indépendante0Commence à partir de
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]

> **Caractéristiques Observables à froid**
> Chaque abonnement démarre une exécution indépendante
> - Chaque abonné reçoit son propre flux de données
> - Un timer indépendant est démarré pour chaque abonnement. Utilisez `share()` si le partage des données est nécessaire.
>
> > Pour plus d'informations, voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables).

## Différence entre interval() vs timer()

Bien que `interval()` et `timer()` soient similaires, il y a quelques différences importantes.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - Démarrage immédiat, publication continue
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Démarrage après un délai
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Sortie:
// interval: 0  (1(après un délai d'une seconde)
// interval: 1  (2(après un délai d'une seconde)
// timer: 0     (2(après un délai d'une seconde)
// interval: 2  (3(après un délai d'une seconde)
// timer: 1     (3(après un délai d'une seconde)
// timer: 2     (4(après un délai d'une seconde)
```

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Cas d'utilisation pratique

### 1. interrogation de l'API

Appeler l'API à intervalles réguliers pour mettre à jour les données.


```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// 5Chaque secondeAPIInterrogation
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError((error: unknown) => {
    console.error('API Error:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Mise à jour de l'état:', data);
});

// Arrêt si nécessaire
// subscription.unsubscribe();
```

### Compte à rebours

Mettre en place un compte à rebours jusqu'à la limite de temps.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // 10Compte à rebours à partir de secondes
  takeWhile(time => time >= 0) // 0Achèvement automatique avec
);

countdown$.subscribe({
  next: time => console.log(`Temps restant: ${time}secondes`),
  complete: () => console.log('Temps écoulé！')
});

// Sortie (en1(chaque seconde):
// Temps restant: 10secondes
// Temps restant: 9secondes
// ...
// Temps restant: 0secondes
// Temps écoulé！
```

### 3. fonction de sauvegarde automatique

Le contenu du formulaire est automatiquement sauvegardé à intervalles réguliers.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Créer un formulaire
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Veuillez remplir le formulaire';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Après l'arrêt de la saisie3Quelques secondes plus tard, sauvegarde automatique (raccourci à des fins de démonstration)
input$.pipe(
  debounceTime(3000), // 3S'il n'y a pas de saisie pendant quelques secondes
  switchMap(() => {
    const formData = new FormData(form);
    // Pour la démonstration: RéelAPIau lieu dePromiseSimulé en
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Sauvegarde automatique:', result);
});
```

### Affichage de l'horloge en temps réel

Met à jour l'heure actuelle en temps réel.

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Utilisation dans le pipeline.

`interval()` est utilisé comme point de départ pour les pipelines ou comme déclencheur pour le contrôle du temps.


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Erreurs courantes.

### 1. oublier de se désinscrire


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

### 2. les abonnements multiples créent des minuteries indépendantes


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Considérations relatives aux performances.

`interval()` est léger, mais des considérations de performance doivent être prises en compte lors de l'exécution à intervalles courts.

> [!TIP]

> **Conseils d'optimisation** :.
> - Ne lancez pas de processus inutiles (filtrez-les avec `filter()`)
> - Utilisez des intervalles courts (moins de 100 ms) avec précaution
> - Assurer la désinscription.
> - Partagez avec `share()` si plus d'un Observer est nécessaire


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Fonctions de Creation Function liées.


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Résumé

- `interval()` émet des valeurs continues à des intervalles spécifiés
- Délivre des entiers continus à partir de 0
- Pas d'achèvement automatique, doit être désabonné
- Fonctionne comme un Observable froid (timer indépendant par abonnement)
- Idéal pour le polling, l'exécution périodique, le compte à rebours, etc.

## Prochaines étapes.

- [timer() - commence à publier après un délai](/fr/guide/creation-functions/basic/timer)
- [fromEvent() - convertit un événement en Observable](/fr/guide/creation-functions/basic/fromEvent)
- [retour à l'aperçu du système de création de base](/fr/guide/creation-functions/basic/)
