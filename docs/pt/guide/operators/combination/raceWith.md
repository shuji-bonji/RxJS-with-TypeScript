---
description: "raceWith é um operador de junção RxJS que adota apenas o fluxo que emitiu o valor primeiro do Observable original e de outros Observables. É ideal para implementações de tempo limite, processos de fallback, adoção mais rápida de várias fontes de dados e outras situações em que a seleção por condições de corrida é necessária; a versão do operador pipe é conveniente para uso em pipelines."
---

# raceWith - adota o fluxo mais rápido

O operador `raceWith` adota apenas o primeiro Observable que emite um valor a partir do Observable original e dos outros Observables especificados,
depois disso, ignora os outros Observable.
Essa é a versão do Pipeable Operator do Creation Function's `race`.

## Sintaxe básica e uso

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentamente. (5Segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2Segundos)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Saída: Rápido (2Segundos)
```

- Somente o Observable que emitiu o valor primeiro (`fast$` neste exemplo) é o vencedor e continua com os fluxos subsequentes.
- Outros Observable são ignorados.

[🌐 Documentação oficial do RxJS - raceWith](https://rxjs.dev/api/operators/raceWith)

## Padrão de utilização típico.

- Implementação de timeout**: competir com o cronômetro de timeout contra o processo principal.
- Processamento de fallback**: adotar o mais rápido de várias fontes de dados
- Interação com o usuário**: adote o clique mais rápido e o progresso automático

## Exemplos práticos de código (com UI)

Este é um exemplo de uma disputa entre o clique manual e o cronômetro de progresso automático e a adoção do mais rápido.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Criação da área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Exemplos práticos de:</h3>';
document.body.appendChild(output);

// Criação de botões
const button = document.createElement('button');
button.textContent = 'Proceder manualmente (5Clique em segundos)';
document.body.appendChild(button);

// Mensagem de espera
const waiting = document.createElement('div');
waiting.textContent = '5Clique no botão dentro de segundos ou aguarde o avanço automático...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Fluxo de clique manual
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 O clique manual foi selecionado！')
);

// Temporizador de progresso automático (5(após 2 segundos)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ O progresso automático foi selecionado！')
);

// Execução da corrida
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- O clique manual é adotado se o botão for clicado em até 5 segundos.
- Após 5 segundos, a progressão automática é adotada.
- O mais rápido é o vencedor**, o mais lento é ignorado.

## 🔄 Diferenças com a Creation Function race.

### Diferenças básicas.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentamente. (5Segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2Segundos)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Saída: Rápido (2Segundos)
```

### Exemplos específicos de uso

**Se você quiser apenas uma corrida simples, a Creation Function é o caminho a seguir**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Servidor1Resposta do'));
const server2$ = timer(2000).pipe(map(() => 'Servidor2Resposta do'));
const server3$ = timer(4000).pipe(map(() => 'Servidor3Resposta do'));

// Simples e fácil de ler
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adotada:', response);
});
// Saída: Adotada: Servidor2Resposta do (Mais rápida2Segundos)
```

**Se você quiser adicionar um processo de conversão ao fluxo principal, o Pipeable Operator é recomendado**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Pesquisa';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Principal: Solicitações de pesquisa do usuário
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Pesquisa...';

    // APISimular uma chamada (3(leva segundos)
    return timer(3000).pipe(
      map(() => '🔍 Resultados da pesquisa: 100Resultados'),
      catchError((err: unknown) => of('❌ Erro ocorrido'))
    );
  })
);

// ✅ Pipeable OperatorEdição - Concluído em um pipeline
userSearch$
  .pipe(
    raceWith(
      // Tempo limite (em2segundos)
      timer(2000).pipe(
        map(() => '⏱️ Tempo limite (s): A pesquisa está demorando muito')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEdição - O mainstream precisa ser escrito separadamente
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Tempo limite (s): A pesquisa está demorando muito')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementação do processamento de fallback**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICriação
const output = document.createElement('div');
output.innerHTML = '<h3>Aquisição de dados (com fall-back)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Início da aquisição de dados';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Durante a aquisição...';

  // PrincipalAPI(Prioridade)：Tempo々Falha
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ PrincipalAPIAquisição bem-sucedida de');
      } else {
        return throwError(() => new Error('PrincipalAPIFracasso'));
      }
    }),
    catchError((err: unknown) => {
      console.log('PrincipalAPIFalha, cedida para fallback...');
      // Atraso por erro, cedido ao fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdição - PrincipalAPIAdicionar fallback a
  mainApi$
    .pipe(
      raceWith(
        // BackupAPI(Fallback)：Um pouco mais lento, mas confiável
        timer(2000).pipe(
          map(() => '🔄 BackupAPIRecuperado de')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Principal') ? 'green' : 'orange';
      }
    });
});
```

**Adotar o mais rápido de várias fontes de dados**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MúltiplosCDNCarga mais rápida de</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Carregar biblioteca';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Carregamento...';

    // CDN1Carregamento (simulado) de
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEdição - CDN1no principal e outrosCDNcomo concorrentes.
    return cdn1$.pipe(
      raceWith(
        // CDN2Carregamento (simulado) de
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Carregamento (simulado) de
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Carregamento concluído</strong><br>
    Adquirido de: ${response.source}<br>
    Arquivo: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Resumo.

- race: ideal se você quiser simplesmente adotar o mais rápido de vários fluxos
- RaceWith: ideal se você quiser implementar tempos limite e fallbacks enquanto transforma e processa o fluxo principal.

## ⚠️ Notas.

### Exemplo de implementação de timeout

Implementação do tratamento de timeout usando raceWith.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Processo demorado (3segundos)
const slowRequest$ = of('Aquisição de dados bem-sucedida').pipe(delay(3000));

// Tempo limite (em2segundos)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Tempo limite (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Saída: Tempo limite (s)
```

### Todos os fluxos estão inscritos em

O raceWith inscreve todos os Observable até que um vencedor seja decidido.
Depois que o vencedor é decidido, o Observable perdedor é automaticamente cancelado.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ O disparo')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ O disparo')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Saída:
// fast$ O disparo
// fast
// (slow$é1Não registrado no segundo,3não é disparado no final do segundo período.)
```

### Para Observable síncrono.

Se tudo for emitido de forma síncrona, o primeiro registrado será o vencedor.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Saída: A (porque foi assinado pela primeira vez)
```

## 📚 Operadores relacionados.

- **[race](/pt/guide/creation-functions/selection/race)** - Versão da Creation Function
- **[timeout](/pt/guide/operators/utility/timeout)** - Operador somente de timeout.
- **[mergeWith](/pt/guide/operators/combination/mergeWith)** - Mesclar todos os fluxos
