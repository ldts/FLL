#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>

#include "pipeline.h"
#include "time_utils.h"

static void *stage_worker(void *arg)
{
	struct stage *step = arg;
	int ret;

	for (;;) {
		ret = sem_wait(&step->nowait);
		if (ret)
			printf("step %d wait error %d.\n",step->params.nth_stage, ret);

		if (step->ops->input)
			ret = step->ops->input(step, NULL);
		if (ret)
			printf("step %d input error %d.\n", step->params.nth_stage, ret);

		ret = step->ops->run(step);
		if (ret)
			printf("step %d run error %d.\n", step->params.nth_stage, ret);

		sem_post(&step->done);
	}
	if (ret < 0)
		printf("step %d failed.\n", step->params.nth_stage);

	return NULL;
}

void stage_up(struct stage *stg, struct stage_params *p, struct stage_ops *o, struct pipeline *pipe)
{
	pthread_attr_t attr;

	stg->pipeline = pipe;
	stg->next = NULL;
	stg->params = *p;
	stg->ops = o;

	sem_init(&stg->nowait, 0, 0);
	sem_init(&stg->done, 0, 0);

	pthread_mutex_init(&stg->lock, NULL);
	pthread_cond_init(&stg->sync, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&stg->worker, &attr, stage_worker, stg);
	pthread_attr_destroy(&attr);
}

void stage_go(struct stage *stg)
{
	sem_post(&stg->nowait);
}

void stage_wait(struct stage *stg)
{
	int ret = sem_wait(&stg->done);
	if (ret)
		printf("%s: step %d wait error %d.\n",
		       __func__, stg->params.nth_stage, ret);
}

int stage_output(struct stage *stg, void *it)
{
	struct stage *next = stg->next;

	if (!next)
		return 0;

	pthread_mutex_lock(&next->lock);
	next->params.data_in = it;
	pthread_cond_signal(&next->sync);
	pthread_mutex_unlock(&next->lock);

	return 0;
}

int stage_input(struct stage *stg, void **it)
{
	pthread_mutex_lock(&stg->lock);
	while (stg->params.data_in == NULL)
		pthread_cond_wait(&stg->sync, &stg->lock);

	*it = stg->params.data_in;
	pthread_mutex_unlock(&stg->lock);

	return 0;
}
	
void stage_down(struct stage *stg)
{
	pthread_cancel(stg->worker);
	pthread_join(stg->worker, NULL);
	pthread_cond_destroy(&stg->sync);
	pthread_mutex_destroy(&stg->lock);
	sem_destroy(&stg->nowait);
	sem_destroy(&stg->done);
}

void stage_printstats(struct stage *stg)
{
	return;
}

void pipeline_init(struct pipeline *pipe)
{
	memset(pipe->stgs, 0, PIPELINE_MAX_STAGE);
	pipe->status = 0;
	pipe->count = 0;
}

int pipeline_register(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;

	pipe->stgs[stg->params.nth_stage] = stg;

	if (stg->params.nth_stage <= 0)
		goto done;

	if (stg->params.nth_stage <= PIPELINE_MAX_STAGE)
		pipe->stgs[stg->params.nth_stage-1]->next = stg;
done:
	++(pipe->count);

	return 0;	
}

int pipeline_deregister(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;
	
	pipe->stgs[stg->params.nth_stage] = NULL;
	--(pipe->count);

	return 0;	
}

int pipeline_run(struct pipeline *pipe)
{
	struct stage *s;
	int ret, n;

	for (n = CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {

		s = pipe->stgs[n];
		s->ops->go(s);

		ret = sem_wait(&s->done);
		if (ret) {
			printf("step %d done error %d.\n", s->params.nth_stage, ret);
			return -EIO;
		}

		if (!s->params.data_out)
		    break;

		if (s->ops->output && s->next) 
			s->ops->output(s, s->params.data_out);
	}

	return 0;
}

void pipeline_terminate(struct pipeline *pipe, int reason)
{
	pipe->status = STAGE_ABRT;
}

int pipeline_pause(struct pipeline *pipe)
{
	struct stage *s;
	int n, ret = 0;
	
	for (n = CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
		s = pipe->stgs[n];
		s->ops->wait(s);
	}

	return ret;
}

int pipeline_getcount(struct pipeline *pipe)
{
	return pipe->count;
}

void pipeline_teardown(struct pipeline *pipe)
{
	struct stage *s;
	int n;
	
	for (n = CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
		s = pipe->stgs[n];
		if (s && s->self) {
			printf("%s: run stage %d.\n", __func__, s->params.nth_stage);
			s->ops->down(s);
		}
	}
}
