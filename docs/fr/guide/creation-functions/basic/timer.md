---
description: "timer() - Fonction de création qui commence à émettre après un délai spécifié : Parfait pour l'exécution différée, le polling avec délai et les implémentations de timeout"
---

# timer() - Commencer à émettre après un délai

`timer()` est une fonction de création qui commence à émettre des valeurs après un délai spécifié, prenant en charge les émissions uniques et périodiques.

## Vue d'ensemble

`timer()` est une fonction de création flexible qui vous permet de contrôler le moment de la première émission. Son comportement change en fonction du nombre d'arguments, et elle peut être utilisée pour une émission ponctuelle ou périodique comme `interval()`.

**Signature** :
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Documentation officielle** : [📘 RxJS Official : timer()](https://rxjs.dev/api/index/function/timer)

## Utilisation de base

Le comportement de `timer()` dépend du nombre d'arguments.

### Émission unique

Si seul le premier argument est spécifié, il émet 0 après le temps spécifié et se termine.

```typescript
import { timer } from 'rxjs';

// Émettre 0 après 3 secondes et terminer
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Valeur:', value),
  complete: () => console.log('Terminé')
});

// Sortie après 3 secondes:
// Valeur: 0
// Terminé
```

### Émission périodique

Si un intervalle est spécifié pour le deuxième argument, il continuera à émettre périodiquement après le délai initial.

```typescript
import { timer } from 'rxjs';

// Démarrer après 3 secondes, puis émettre des valeurs toutes les 1 seconde
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Valeur:', value));

// Sortie:
// Valeur: 0  (après 3 secondes)
// Valeur: 1  (après 4 secondes)
// Valeur: 2  (après 5 secondes)
// ... (continue indéfiniment)
```

## Caractéristiques importantes

### 1. Spécification flexible des délais

Le délai peut être spécifié sous forme d'un nombre en millisecondes ou d'un objet `Date`.

```typescript
import { timer } from 'rxjs';

// Spécifier en millisecondes
timer(5000).subscribe(() => console.log('Après 5 secondes'));

// Spécifier avec un objet Date (exécuter à une heure spécifique)
const targetTime = new Date(Date.now() + 10000); // 10 secondes plus tard
timer(targetTime).subscribe(() => console.log('Exécuter à l\'heure spécifiée'));
```

### 2. Le comportement change selon le deuxième argument

Que le deuxième argument soit spécifié ou non détermine si l'Observable se termine.

```typescript
import { timer } from 'rxjs';

// Sans deuxième argument - émet une fois et termine
timer(1000).subscribe({
  next: value => console.log('Une fois:', value),
  complete: () => console.log('Terminé')
});

// Avec deuxième argument - émet indéfiniment
timer(1000, 1000).subscribe({
  next: value => console.log('Répéter:', value),
  complete: () => console.log('Terminé (non affiché)')
});
```

> [!IMPORTANT]
> **Avec le deuxième argument, il ne se termine pas**
>
> Si vous spécifiez le deuxième argument comme `timer(1000, 1000)`, il continuera à émettre indéfiniment, tout comme `interval()`. Le désabonnement est toujours nécessaire.

### 3. Cold Observable

`timer()` est un Cold Observable, ce qui signifie qu'un timer indépendant est créé pour chaque abonnement.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Démarrage');

// Abonnement 1
timer$.subscribe(() => console.log('Observateur 1'));

// Ajout de l'abonnement 2 après 500ms
setTimeout(() => {
  timer$.subscribe(() => console.log('Observateur 2'));
}, 500);

// Sortie:
// Démarrage
// Observateur 1  (après 1 seconde)
// Observateur 2  (après 1.5 secondes - timer indépendant)
```

> [!NOTE]
> **Caractéristiques du Cold Observable** :
> - Une exécution indépendante est lancée pour chaque abonnement
> - Chaque abonné reçoit son propre flux de données
> - Un timer indépendant est démarré pour chaque abonnement ; comme pour `interval()`, utilisez `share()` si le partage est nécessaire
>
> Voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables) pour plus d'informations.

## Différence entre timer() et interval()

La principale différence entre les deux est le moment de la première émission.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Démarrage');

// interval() - démarre immédiatement (première valeur après 1 seconde)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - sans délai (première valeur immédiatement)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - démarre après un délai de 2 secondes
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(délai):', value);
});
```

| Fonction de création | Moment de la première émission | Objectif |
|-------------------|----------------------|---------|
| `interval(1000)` | Après 1 seconde | Démarrer l'exécution périodique immédiatement |
| `timer(0, 1000)` | Immédiatement | Vouloir la première exécution immédiatement |
| `timer(2000, 1000)` | Après 2 secondes | Exécution périodique après délai |
| `timer(2000)` | Après 2 secondes (une seule fois) | Exécution différée (unique) |

## Cas d'utilisation pratiques

### 1. Exécution différée

Exécuter un processus une seule fois après une certaine période de temps.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Données récupérées après 2 secondes:', data);
});
```

### 2. Polling avec délai

Commencer le polling après une certaine période au lieu d'exécuter immédiatement la première fois.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Commencer le polling après 5 secondes, puis toutes les 10 secondes
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Réessayer jusqu'à 3 fois en cas d'erreur
);

const subscription = polling$.subscribe(data => {
  console.log('Mise à jour du statut:', data);
});

// Arrêter si nécessaire
// subscription.unsubscribe();
```

### 3. Traitement de timeout

Le timeout se produit lorsque le traitement n'est pas terminé dans une certaine période.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Utiliser celui qui arrive en premier
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Données récupérées:', data),
  error: err => console.error('Erreur:', err.message)
});
```

### 4. Masquage automatique des notifications

Masquer automatiquement les notifications après une certaine période suivant leur affichage.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Afficher notification:', notification.message);

    // Masquage automatique après 5 secondes
    return timer(5000).pipe(
      takeUntil(dismiss$), // Annuler si fermé manuellement
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Masquer notification:', id);
});

// Afficher une notification
notifications$.next({ id: 1, message: 'Nouveau message reçu' });

// Pour fermer manuellement
// dismiss$.next(1);
```

## Utilisation dans un pipeline

`timer()` est utilisé comme point de départ pour un traitement différé ou une exécution périodique.

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// Compte à rebours (de 10 secondes à 0 seconde)
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // De 0 à 10 (11 valeurs)
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`Restant: ${time} secondes`),
  complete: () => console.log('Timer terminé')
});

// Sortie:
// Restant: 10 secondes
// Restant: 9 secondes
// ...
// Restant: 0 secondes
// Timer terminé
```

## Erreurs courantes

### 1. Oublier de se désabonner avec le deuxième argument

```typescript
// ❌ Incorrect - s'exécute indéfiniment avec le deuxième argument
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Valeur:', value); // S'exécute pour toujours
  });
}

startTimer();

// ✅ Correct - conserver l'abonnement et se désabonner si nécessaire
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Auto-complétion après 10 fois
  ).subscribe(value => {
    console.log('Valeur:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. Ne pas comprendre la différence avec interval()

```typescript
// ❌ Confusion - interval() démarre immédiatement (première valeur après 1 seconde)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 0 émis après 1 seconde
});

// ✅ timer() - quand vous voulez émettre la première valeur immédiatement sans délai
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 0 émis immédiatement
});
```

## Considérations sur les performances

Bien que `timer()` soit léger, son utilisation peut affecter les performances.

> [!TIP]
> **Conseils d'optimisation** :
> - Ne pas spécifier de deuxième argument pour une exécution unique
> - Toujours se désabonner quand ce n'est plus nécessaire
> - Si plusieurs Observateurs sont nécessaires, les partager avec `share()`
> - Utiliser les intervalles courts (moins de 100ms) avec prudence

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Problème de performance - plusieurs timers indépendants
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observateur 1:', value));
timer$.subscribe(value => console.log('Observateur 2:', value));
// Deux timers s'exécutent en parallèle

// ✅ Optimisation - partager un seul timer
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observateur 1:', value));
sharedTimer$.subscribe(value => console.log('Observateur 2:', value));
// Un seul timer est partagé
```

## Fonctions de création associées

| Fonction | Différence | Utilisation |
|----------|------|----------|
| **[interval()](/fr/guide/creation-functions/basic/interval)** | Démarre immédiatement (pas de délai) | Exécution périodique sans délai |
| **[of()](/fr/guide/creation-functions/basic/of)** | Émet de façon synchrone et immédiate | Quand l'asynchrone n'est pas nécessaire |
| **defer()** | Différer le traitement jusqu'à l'abonnement | Génération de valeurs dynamiques |

## Résumé

- `timer()` est une fonction de création qui commence à émettre après un délai
- Sans deuxième argument : émission unique (se termine)
- Avec deuxième argument : émission périodique (ne se termine pas)
- Le délai peut être spécifié en millisecondes ou en tant qu'objet `Date`
- Idéal pour l'exécution différée, le polling avec délai, le traitement de timeout

## Prochaines étapes

- [interval() - Émission continue à intervalles spécifiés](/fr/guide/creation-functions/basic/interval)
- [defer() - Différer la génération jusqu'à l'abonnement](/fr/guide/creation-functions/conditional/defer)
- [Retour aux fonctions de création de base](/fr/guide/creation-functions/basic/)
